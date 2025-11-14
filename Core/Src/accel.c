#include "stm32g4xx_hal.h"
#include "accel.h"
#include "stdio.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_spi.h"
#include <stdint.h>
#include <string.h>

/* Write 1 byte to a register address */
HAL_StatusTypeDef accel_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data){
  uint8_t buffer[2] = {ACCEL_CMD_WRITE(reg), data};
  
  ACCEL_CS_LOW();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, buffer, sizeof(buffer), 1);
  ACCEL_CS_HIGH();

  return status;
}

/* Single Byte Read from given register, must ignore dummy byte */
HAL_StatusTypeDef accel_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data){
  if (!data) return HAL_ERROR;

  uint8_t tx[3] = {ACCEL_CMD_READ(reg), 0x00, 0x00};
  uint8_t rx[3] = {0};

  ACCEL_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, sizeof(tx), 50);
  ACCEL_CS_HIGH();
  if (st != HAL_OK) return st;
  
  *data = rx[2];
  return st;
}

/* Burst read function using auto-increment for BMI-088 */
HAL_StatusTypeDef accel_read_buffer(SPI_HandleTypeDef *hspi, uint8_t start_reg,
                                    uint8_t *dst, uint16_t len) {
  if (!dst || !len) return HAL_ERROR;

  uint8_t tx[8] = {[0] = ACCEL_CMD_READ(start_reg)};
  memset(&tx[1], 0x00, sizeof(tx) - 1);
  uint8_t rx[8];

  ACCEL_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, sizeof(tx), HAL_MAX_DELAY);
  ACCEL_CS_HIGH();

  if (st == HAL_OK) memcpy(dst, &rx[2], len);
  return st;
}

/* Performs self-test, writes data to out, and reinitializes the device. */
HAL_StatusTypeDef accel_selftest(SPI_HandleTypeDef *hspi, accel_data_t *out) {
  accel_data_t data_p;
  accel_data_t data_n;

  accel_write_reg(hspi, ACCEL_CONF, ACC_TEST_CONF);
  HAL_Delay(5);

  accel_write_reg(hspi, ACC_SELF_TEST, ACC_POS_POL);
  HAL_Delay(55);
  accel_read(hspi, &data_p);

  accel_write_reg(hspi, ACC_SELF_TEST, ACC_NEG_POL);
  HAL_Delay(55);
  accel_read(hspi, &data_n);

  accel_write_reg(hspi, ACC_SELF_TEST, ACC_TEST_OFF);
  out->x = data_p.x - data_n.x;
  out->y = data_p.y - data_n.y;
  out->z = data_p.z - data_n.z;
  
  return accel_init(hspi);
}

/* Configure the accelerometer */
HAL_StatusTypeDef accel_init(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef status;
  uint8_t id = 0;

  /* Dummy read */ 
  status = accel_read_reg(hspi, ACCEL_CHIP_ADDR, &id);

  /* WHO_AM_I should be 0x1E at 0x00 (taken directly from Gyro Driver) */ 
  status = accel_read_reg(hspi, ACCEL_CHIP_ADDR, &id);
  if (status != HAL_OK) return status;
  if (id != ACCEL_CHIP_ID) {
    printf("Accel WHOAMI mismatch: 0x%02X (exp 0x1E)\n", id);
    return HAL_ERROR;
  }

  /* Soft reset */
  status = accel_write_reg(hspi, ACCEL_RESET, ACCEL_RESET_VAL);
  if (status != HAL_OK) return status;
  HAL_Delay(30);

  /* Power on (enter normal mode) */
  status = accel_write_reg(hspi, ACCEL_POWER_CTRL, POWER_ON);
  if (status != HAL_OK) return status;
  HAL_Delay(500);

  /* Set range to ±24g */
  status = accel_write_reg(hspi, ACCEL_RANGE, ACCEL_RANGE_VAL);
  if (status != HAL_OK) return status;
  HAL_Delay(30);

  /* Bandwith of low pass filter config to normal and ODR set to 1600hz */
  status = accel_write_reg(hspi, ACCEL_CONF, ACCEL_CONF_VAL);
  if (status != HAL_OK) return status;

  return HAL_OK;
}

/* Read the accelermoter axis data */
HAL_StatusTypeDef accel_read(SPI_HandleTypeDef *hspi, accel_data_t *accelData){
  uint8_t rxBuffer[ACCEL_BUF_SIZE];
  HAL_StatusTypeDef status = accel_read_buffer(hspi, ACCEL_X_LSB, rxBuffer, ACCEL_BUF_SIZE);
  if (status != HAL_OK){
    return status;
  }

  // Set struct values to raw data from accel
  accelData->x = (int16_t)((uint16_t)rxBuffer[1] << 8 | rxBuffer[0]);
  accelData->y = (int16_t)((uint16_t)rxBuffer[3] << 8 | rxBuffer[2]);
  accelData->z = (int16_t)((uint16_t)rxBuffer[5] << 8 | rxBuffer[4]);
  return HAL_OK;
}

/*
static float accel_sensitivity_mg_per_lsb(AccelRange range)
{
    switch (range) {
    case ACCEL_RANGE_3g:  return 10.923f;
    case ACCEL_RANGE_6g:  return 5.461f;
    case ACCEL_RANGE_12g:  return 2.730f;
    case ACCEL_RANGE_24g: return 1.365f;
    default:                 return 1.365f; // safe fallback
    }
}
*/

void convert_raw_accel_to_mg(accel_data_t *data, float *x, float *y, float *z){
  float mg_per_lsb = 24000.0f / 32768.0f;
  *x = data->x * mg_per_lsb;
  *y = data->y * mg_per_lsb;
  *z = data->z * mg_per_lsb;
}