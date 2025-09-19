#pragma once
#include "stm32g4xx_hal.h"
#include "BMP390.h"

#define BAROMTER_POWER_CONTROl_REGISTER PWR_CTRL
#define BAROMETER_SOFTRESET 0xB6
#define BAROMETER_GPIO_PIN GPIO_PIN_13
#define BAROMETER_GPIO_PORT GPIOB
#define BAROMETER_SPI_WRITE (0 << 7)
#define BAROMETER_SPI_READ (1 << 7)

// Power modes
#define BAROMETER_SLEEP_MODE 0x00
#define BAROMETER_FORCED_MODE 0x01
#define BAROMETER_NORMAL_MODE 0x03

// Pressure oversampling (precision)
#define PRESSURE_RES_ULTRA_LOW 0b000
#define PRESSURE_RES_LOW 0b001
#define PRESSURE_RES_STANDARD 0b010
#define PRESSURE_RES_HIGH 0b011
#define PRESSURE_RES_ULTRA_HIGH 0b100
#define PRESSURE_RES_HIGHEST 0b101

// Read / write
#define BAROMETER_INITIALIZATION_TIMEOUT 50U
#define BAROMETER_READ_BIT 0x80
#define BAROMETER_READ_TIMEOUT 10U

HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef barometer_read_pressure(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *out_data, uint16_t out_len);