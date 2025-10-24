#include "gyro.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include <stdio.h>
#include <string.h>

// inline functions

static void gyro_cs_low() {
  HAL_GPIO_WritePin(gyro_CS_GPIO_Port, gyro_CS_Pin, GPIO_PIN_RESET);
}

static void gyro_cs_high() {
  HAL_GPIO_WritePin(gyro_CS_GPIO_Port, gyro_CS_Pin, GPIO_PIN_SET);
}

/* These functions should hopefully correctly structure the messages */
static inline uint8_t GYRO_CMD_WRITE(uint8_t reg) {
  return (uint8_t)((reg << 1) | 0x00u);
}
static inline uint8_t GYRO_CMD_READ(uint8_t reg) {
  return (uint8_t)((reg << 1) | 0x01u);
}

// (Optional) tiny CS settle; keep if your board needs it
static inline void cs_low_settle(void) {
  gyro_cs_low();
  for (volatile int i = 0; i < 50; ++i) {
    __NOP();
  }
}

// ---- Single-register write ----
HAL_StatusTypeDef gyro_write_register(SPI_HandleTypeDef *hspi, uint8_t reg,
                                      uint8_t value) {
  uint8_t tx[2] = {GYRO_CMD_WRITE(reg), value};
  cs_low_settle();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
  gyro_cs_high();
  return st;
}

// ---- Single-register read (no special “read bit” in the MSB, no accel-style
// dummy rules) ----
HAL_StatusTypeDef gyro_read_register(SPI_HandleTypeDef *hspi, uint8_t reg,
                                     uint8_t *value) {
  if (!value)
    return HAL_ERROR;
  uint8_t tx[2] = {GYRO_CMD_READ(reg), 0x00};
  uint8_t rx[2] = {0, 0};
  cs_low_settle();
  HAL_StatusTypeDef st =
      HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
  gyro_cs_high();
  if (st == HAL_OK)
    *value = rx[1]; // data clocks out in the second byte
  return st;
}
uint8_t bmi088g_probe_id(SPI_HandleTypeDef *hspi) {
  uint8_t cmd = (0x00u) | 0x01u; // CHIP_ID read
  uint8_t id = 0x8a;
  gyro_cs_low();
  HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(hspi, &id, 1, HAL_MAX_DELAY);
  gyro_cs_high();
  return id;
}
// ---- Burst read (address auto-increments while CS stays low) ----
HAL_StatusTypeDef gyro_read_buffer(SPI_HandleTypeDef *hspi, uint8_t start_reg,
                                   uint8_t *dst, uint16_t len) {
  if (!dst || !len)
    return HAL_ERROR;
  uint8_t cmd = GYRO_CMD_READ(start_reg);
  cs_low_settle();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
  if (st == HAL_OK)
    st = HAL_SPI_Receive(hspi, dst, len, HAL_MAX_DELAY);
  gyro_cs_high();
  return st;
}

// After MX_GPIO_Init(); HAL_Delay(20); MX_SPI1_Init();

HAL_StatusTypeDef gyro_init(SPI_HandleTypeDef *hspi) {

  if (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY) {
    return HAL_BUSY;
  }

  // read chip ID
  uint8_t id;
  uint8_t data = bmi088g_probe_id(hspi);
  HAL_StatusTypeDef st = gyro_read_register(hspi, GYRO_CHIP_ID, &id);
  if (st != HAL_OK) {

    return st;
  }
  if (id !=
      GYRO_CHIP_ID_VALUE) // check the register value not the register address.
  {
    while (1) {
      printf("Gyro read chip id failed!: hex id 0x%02X\n", data);
      HAL_Delay(500);
    }
    return HAL_ERROR;
  }

  // soft reset
  gyro_write_register(hspi, GYRO_SOFTRESET, 0xB6);
  HAL_Delay(50);

  // normal mode
  gyro_write_register(hspi, GYRO_LPM1, 0x00);

  // full-scale range
  gyro_write_register(hspi, GYRO_RANGE, GYRO_RANGE_2000DPS);

  // full bandwidth
  gyro_write_register(hspi, GYRO_BANDWIDTH, 0x00);

  return HAL_OK;
}

// this uses the new helper to read multiple bytes
HAL_StatusTypeDef gyro_read(SPI_HandleTypeDef *hspi, gyro_data_t *gyro_data) {
  uint8_t buf[6];
  HAL_StatusTypeDef st = gyro_read_buffer(hspi, RATE_X_LSB, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  // LSB first, then MSB
  gyro_data->rate_x = (int16_t)((buf[1] << 8) | buf[0]);
  gyro_data->rate_y = (int16_t)((buf[3] << 8) | buf[2]);
  gyro_data->rate_z = (int16_t)((buf[5] << 8) | buf[4]);
  return HAL_OK;
}
