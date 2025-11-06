/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "barometer.h"
#include "cmsis_os2.h"
#include "gyro.h"
#include "stm32g4xx_hal_gpio.h"
#include "task.h"
#include "telemetry.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Debug prints */
// #define DEBUG_PRINTS

/* ====================== Helpers & Globals ====================== */

SPI_HandleTypeDef hspi1;
static osMutexId_t cdc_mutex;
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Convert milliseconds to RTOS ticks (works for any tick freq) */
#define MS_TO_TICKS(ms)                                                        \
  ((uint32_t)((((uint64_t)(ms)) * osKernelGetTickFreq()) / 1000u))

/* Task handles */
osThreadId_t TaskHandles[2];
/* ---------------- Task attributes (sizes in bytes) ----------------
   Start generously to avoid stack overflows from drivers/printf.
   Tune down later with uxTaskGetStackHighWaterMark() if desired.
*/
enum {
  STACK_DEFAULT = 512,
  STACK_SENSOR = 1024 * 4,
  STACK_DISPATCH = 1024 * 4
};

static const osThreadAttr_t sensorTask_attributes = {
    .name = "SensorTask",
    .priority = osPriorityNormal,
    .stack_size = STACK_SENSOR};
static const osThreadAttr_t loggingTask_attributes = {
    .name = "LoggingTask",
    /* Make dispatcher a bit higher so it can drain queues promptly */
    .priority = osPriorityNormal + 1,
    .stack_size = STACK_DISPATCH};

/* ====================== Prototypes ====================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

static void SetupSystem(void *argument);
static void SensorTask(void *arg);
static void DispatchTask(void *arg);
void cdc_printf_init(void);
uint8_t router_ready = 0;
/* ====================== Main ====================== */

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();

  /* Initialize RTOS kernel objects first */
  osKernelInitialize();

  /* Create threads */
  TaskHandles[0] = osThreadNew(SensorTask, NULL, &sensorTask_attributes);
  TaskHandles[1] = osThreadNew(DispatchTask, NULL, &loggingTask_attributes);
  SetupSystem(NULL); // Call setup task directly before starting scheduler
  if (!TaskHandles[0] || !TaskHandles[1]) {
    /* Not enough heap or configTOTAL_HEAP_SIZE too small */
    Error_Handler();
  }

  /* Start scheduler */
  osKernelStart();

  /* Should never reach here */
  while (1) {
  }
}

/* ====================== Clocks / Peripherals ====================== */

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void) {
  /* NOTE: Ensure HAL_SPI_MspInit() configures SCK/MISO/MOSI AF pins
     in stm32g4xx_hal_msp.c, or SPI will not drive pins. */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* CS pins high */
  HAL_GPIO_WritePin(GPIOB, accel_CS_Pin | gyro_CS_Pin | baro_CS_Pin,
                    GPIO_PIN_SET);
  /* LED low */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /* CS pins */
  GPIO_InitStruct.Pin = accel_CS_Pin | gyro_CS_Pin | baro_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* LED */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);
}

/* ====================== Tasks ====================== */

/* CubeMX-style: do USB init in a “default” task after the scheduler starts.
   Then initialize the telemetry router and release a semaphore
   so other tasks can proceed. */
static void SetupSystem(void *argument) {
  (void)argument;
  /* Initialize USB device (creates RTOS resources internally) */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
  MX_USB_Device_Init();
  cdc_printf_init();

  /* Initialize telemetry router AFTER USB is up (if it needs endpoints) */
}
static void init_router() {
  SedsResult r = init_telemetry_router();
  if (r != SEDS_OK) {
    while (1) {
      print_telemetry_error(r);
    }
    /* still continue to unlock, or block forever? choose to continue */
  }
  router_ready = 1;
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
}

static void SensorTask(void *arg) {
  (void)arg;

  /* Wait until router is ready (and USB initialized) */

  while (!router_ready) {
    osDelay(MS_TO_TICKS(10));
  }
  if (gyro_init(&hspi1) != HAL_OK) {
    die("gyro init failed\r\n");
  }
  if (init_barometer(&hspi1) != HAL_OK) {
    die("barometer init failed\r\n");
  }

  float barometer_pressure[3] = {100.0f, 100.0f, 100.0f};
  gyro_data_t data = {0, 0, 0};

  for (;;) {
    /* Read barometer */
    HAL_StatusTypeDef st = get_temperature_pressure_altitude_non_blocking(
        &hspi1, &barometer_pressure[1], &barometer_pressure[0],
        &barometer_pressure[2]);
    if (st != HAL_OK) {
      printf("barometer read failed: %d\r\n", st);
    }

    /* Read gyro */
    st = gyro_read(&hspi1, &data);
    if (st != HAL_OK) {
      printf("gyro read failed: %d\r\n", st);
    }

    /* Log telemetry (async) */
    SedsResult r;
    r = log_telemetry_asynchronous(
        SEDS_DT_BAROMETER_DATA, barometer_pressure,
        (uint32_t)(sizeof(barometer_pressure) / sizeof(barometer_pressure[0])),
        (uint32_t)sizeof(barometer_pressure[0]));

    if (r != SEDS_OK) {
      while (1) {
        print_telemetry_error(r);
      }
    }

    float gyro_vals[3];
    gyro_convert_to_dps(&data, &gyro_vals[0], &gyro_vals[1], &gyro_vals[2]);

    r = log_telemetry_asynchronous(
        SEDS_DT_GYROSCOPE_DATA, gyro_vals,
        (uint32_t)(sizeof(gyro_vals) / sizeof(gyro_vals[0])),
        (uint32_t)sizeof(gyro_vals[0]));
    if (r != SEDS_OK) {
      while (1) {
        print_telemetry_error(r);
      }
    }
#ifdef DEBUG_PRINTS
    vPrintHeapStats("SensorTask");
#endif
    /* 500 ms sample period */
    osDelay(MS_TO_TICKS(500));
  }
}

static void DispatchTask(void *arg) {
  (void)arg;
  init_router();

  for (;;) {
/* Drain router queues for ~1000 ms */
#ifdef DEBUG_PRINTS
    vPrintHeapStats("dispatch_task");
#endif
    SedsResult r = process_all_queues_timeout(0);
    if (r != SEDS_OK) {
      while (1) {
        print_telemetry_error(r);
      }
    }
    /* Yield a bit */
    osDelay(MS_TO_TICKS(50));
  }
}

/* ====================== Error handling & CDC printf ====================== */

void Error_Handler(void) {
  /* Keep IRQs ON so SysTick/RTOS can run if alive */
  /* Blink LED with crude delay (no HAL_Delay reliance) */
  while (1) {
    // HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
    for (volatile uint32_t i = 0; i < 1000000; ++i) {
      printf("Error Handler ahs been triggered\r\n");
    }
  }
}
void cdc_printf_init(void) {
  const osMutexAttr_t attr = {
      .name = "cdc_tx_mutex",
      .attr_bits = 0U,
      .cb_mem = NULL,
      .cb_size = 0U,
  };
  cdc_mutex = osMutexNew(&attr);
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  (void)file;
  (void)line;
  /* Optionally print or blink pattern */
}
#endif

static inline int cdc_in_isr(void) {
  return (__get_IPSR() != 0U); // CMSIS core function
}

/* Helper: is the kernel running? */
static inline int cdc_kernel_running(void) {
  return (osKernelGetState() == osKernelRunning);
}

static void cdc_lock(void) {
  /* Do NOT try to lock from ISR or before scheduler: just drop / best effort */
  if (!cdc_kernel_running() || cdc_in_isr() || cdc_mutex == NULL) {
    return;
  }
  (void)osMutexAcquire(cdc_mutex, osWaitForever);
}

static void cdc_unlock(void) {
  if (!cdc_kernel_running() || cdc_in_isr() || cdc_mutex == NULL) {
    return;
  }
  (void)osMutexRelease(cdc_mutex);
}

// Wait until previous packet is gone (TxState==0)
static void cdc_wait_idle(uint32_t timeout_ms) {
  uint32_t start = HAL_GetTick();

  while (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (!hcdc || hcdc->TxState == 0) {
      break;
    }

    if ((HAL_GetTick() - start) >= timeout_ms) {
      /* Give up after timeout; avoid hard lock if USB wedges */
      break;
    }

    /* If kernel running, yield; otherwise busy-wait */
    if (cdc_kernel_running()) {
      osDelay(1);
    } else {
      HAL_Delay(1);
    }
  }
}

// Send buffer in 64B chunks, honor BUSY, and ZLP if len%64==0
static void cdc_write_raw(const uint8_t *buf, uint16_t len) {
  if (!buf || !len) {
    return;
  }

  /* Do NOT attempt CDC TX from ISR; just drop to avoid deadlocks. */
  if (cdc_in_isr()) {
    return;
  }

  /* If USB not configured, silently drop (printf as best-effort debug) */
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
    return;
  }

  cdc_lock();

  uint16_t sent = 0;
  while (sent < len) {
    uint16_t chunk = MIN(64, (uint16_t)(len - sent));
    USBD_StatusTypeDef st;
    uint32_t start = HAL_GetTick();

    /* Try to submit this chunk, with timeout if BUSY */
    do {
      st = CDC_Transmit_FS((uint8_t *)&buf[sent], chunk);
      if (st == USBD_BUSY) {
        if ((HAL_GetTick() - start) >= 1000U) { // 1s timeout
          /* Give up on this chunk; avoid hard lock */
          cdc_unlock();
          return;
        }
        if (cdc_kernel_running()) {
          osDelay(1);
        } else {
          HAL_Delay(1);
        }
      }
    } while (st == USBD_BUSY);

    /* If error or not configured anymore, bail out */
    if (st != USBD_OK || hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
      cdc_unlock();
      return;
    }

    /* Wait until IN transfer completes before next packet, bounded time */
    cdc_wait_idle(1000U); // 1s max
    sent += chunk;
  }

  // Zero-Length Packet if exactly multiple of 64 (forces flush on host)
  if ((len & 0x3F) == 0) {
    USBD_StatusTypeDef st;
    uint32_t start = HAL_GetTick();
    do {
      st = CDC_Transmit_FS(NULL, 0); // ZLP
      if (st == USBD_BUSY) {
        if ((HAL_GetTick() - start) >= 1000U) {
          break; // give up
        }
        if (cdc_kernel_running()) {
          osDelay(1);
        } else {
          HAL_Delay(1);
        }
      }
    } while (st == USBD_BUSY);

    cdc_wait_idle(1000U);
  }

  cdc_unlock();
}

#ifdef __GNUC__
int _write(int file, char *ptr, int len) {
  (void)file;
  if (len <= 0)
    return 0;

  // Convert \n -> \r\n into a small rolling buffer
  uint8_t buf[128];
  int i = 0;
  while (i < len) {
    int w = 0;
    while (i < len && w < (int)sizeof(buf) - 1) {
      uint8_t c = (uint8_t)ptr[i++];
      if (c == '\n' && w < (int)sizeof(buf) - 2)
        buf[w++] = '\r';
      buf[w++] = c;
    }
    cdc_write_raw(buf, (uint16_t)w);
  }
  return len;
}
#else
int fputc(int ch, FILE *f) {
  (void)f;
  uint8_t two[2];
  uint16_t n = 0;
  if (ch == '\n')
    two[n++] = '\r';
  two[n++] = (uint8_t)ch;
  cdc_write_raw(two, n);
  return ch;
}
#endif