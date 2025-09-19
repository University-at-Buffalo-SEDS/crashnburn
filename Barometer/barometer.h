#pragma once
#include "stm32g4xx_hal.h"
#include "BMP390.h"

#define BAROMTER_POWER_CONTROl_REGISTER 0x1B
#define BAROMETER_PRESSURE_REGISTER 
#define BAROMETER_GPIO_PIN GPIO_PIN_13
#define BAROMETER_GPIO_PORT GPIOB
#define BAROMETER_NORMAL_MODE 0x03
#define BAROMETER_INITIALIZATION_TIMEOUT 50U
#define BAROMETER_READ_BIT 0x80
#define BAROMETER_READ_TIMEOUT 10U


HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef barometer_read_pressure(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *out_data, uint16_t out_len);