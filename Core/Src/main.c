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
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "gyro.h"
#include "task.h"
#include "telemetry.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* Definitions for defaultTask */
osThreadId_t TaskHandles[3];
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4};
const osThreadAttr_t sensorTask_attributes = {
    .name = "SensorTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4};
const osThreadAttr_t loggingTask_attributes = {
    .name = "LoggingTask",
    .priority = (osPriority_t)osPriorityIdle + 1,
    .stack_size = 128 * 4};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
static void SensorTask(void *arg);
static void DispatchTask(void *arg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  SedsResult r = init_telemetry_router();
  if (r != SEDS_OK) {
    print_telemetry_error(r);
  }

  if (gyro_init(&hspi1) != HAL_OK) {
    die("gyro init failed\r\n");
  }
  if (init_barometer(&hspi1) != HAL_OK) {

    die("barometer init failed\r\n");
  }

  // ---- Create tasks ----
  TaskHandles[0] = osThreadNew(SensorTask, NULL, &sensorTask_attributes);
  TaskHandles[1] = osThreadNew(DispatchTask, NULL, &loggingTask_attributes);
  //TaskHandles[2] = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  // Check task creation
  // ---- Go! ----
  osKernelStart();
  while (1) {
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
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

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
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
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, accel_CS_Pin | gyro_CS_Pin | baro_CS_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : accel_CS_Pin gyro_CS_Pin baro_CS_Pin */
  GPIO_InitStruct.Pin = accel_CS_Pin | gyro_CS_Pin | baro_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END 5 */
}
static void SensorTask(void *arg) {
  (void)arg;

  float barometer_pressure[3] = {100.0f, 100.0f, 100.0f};
  gyro_data_t data;

  for (;;) {
    // ---- Read barometer ----
    HAL_StatusTypeDef st = get_temperature_pressure_altitude_non_blocking(
        &hspi1, &barometer_pressure[1], &barometer_pressure[0],
        &barometer_pressure[2]);
    if (st != HAL_OK) {
      die("barometer read failed: %d\r\n", st);
    }

    // ---- Read gyro ----
    st = gyro_read(&hspi1, &data);
    if (st != HAL_OK) {
      die("gyro read failed: %d\r\n", st);
    }

    // ---- Log telemetry (asynchronous) ----
    SedsResult r;

    r = log_telemetry_asynchronous(
        SEDS_DT_BAROMETER_DATA, barometer_pressure,
        (uint32_t)(sizeof(barometer_pressure) / sizeof(barometer_pressure[0])),
        (uint32_t)sizeof(barometer_pressure[0]));
    if (r != SEDS_OK) {
      print_telemetry_error(r);
    }

    float gyro_vals[3];
    gyro_convert_to_dps(&data, &gyro_vals[0], &gyro_vals[1], &gyro_vals[2]);
    r = log_telemetry_asynchronous(
        SEDS_DT_GYROSCOPE_DATA, gyro_vals,
        (uint32_t)(sizeof(gyro_vals) / sizeof(gyro_vals[0])),
        (uint32_t)sizeof(gyro_vals[0]));
    if (r != SEDS_OK) {
      print_telemetry_error(r);
    }

    // Sample period: 500 ms
    osDelay(pdMS_TO_TICKS(500));
  }
}

static void DispatchTask(void *arg) {
  (void)arg;

  for (;;) {
    // Drain the router queues for 20ms
    SedsResult r = process_all_queues_timeout(20);
    if (r != SEDS_OK) {
      print_telemetry_error(r);
    }

    // Back off a bit to let other tasks run; tune as needed
    osDelay(pdMS_TO_TICKS(5));
  }
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

static inline void cdc_write_blocking(const uint8_t *data, uint16_t len) {
  // Busy-wait with a tiny delay until the CDC endpoint accepts the packet.
  while (CDC_Transmit_FS((uint8_t *)data, len) == USBD_BUSY) {
    osDelay(1);
  }
}

#ifdef __GNUC__
int _write(int file, const char *ptr, int len) {
  (void)file;
  if (len <= 0)
    return 0;

  for (int i = 0; i < len; ++i) {
    uint8_t out[2];
    uint16_t n = 0;
    uint8_t c = (uint8_t)ptr[i];
    if (c == '\n')
      out[n++] = '\r';
    out[n++] = c;
    cdc_write_blocking(out, n);
  }
  return len;
}
#else
int fputc(int ch, FILE *f) {
  (void)f;
  uint8_t out[2];
  uint16_t n = 0;
  if (ch == '\n')
    out[n++] = '\r';
  out[n++] = (uint8_t)ch;
  cdc_write_blocking(out, n);
  return ch;
}
#endif

// Optionally, call this once after USB init:
void cdc_stdio_unbuffered(void) { setvbuf(stdout, NULL, _IONBF, 0); }