#pragma once
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdint.h>

// ----- Registers (unchanged) -----
#define CHIP_ID 0x00
#define REV_ID 0x01
#define ERR_REG 0x02
#define STATUS 0x03
#define DATA_0 0x04
#define DATA_1 0x05
#define DATA_2 0x06
#define DATA_3 0x07
#define DATA_4 0x08
#define DATA_5 0x09
#define SENSORTIME_0 0x0C
#define SENSORTIME_1 0x0D
#define SENSORTIME_2 0x0E
#define EVENT 0x10
#define INT_STATUS 0x11
#define FIFO_LENGTH_0 0x12
#define FIFO_LENGTH_1 0x13
#define FIFO_DATA 0x14
#define FIFO_WTM_0 0x15
#define FIFO_WTM_1 0x16
#define FIFO_CONFIG_0 0x17
#define FIFO_CONFIG_1 0x18
#define INT_CTRL 0x19
#define IF_CONF 0x1A
#define PWR_CTRL 0x1B
#define OSR 0x1C
#define ODR 0x1D
#define CONFIG 0x1F
#define CMD 0x7E

#define NVM_PAR_T1 0x31
#define NVM_PAR_T2 0x33
#define NVM_PAR_T3 0x35
#define NVM_PAR_P1 0x36
#define NVM_PAR_P2 0x38
#define NVM_PAR_P3 0x3A
#define NVM_PAR_P4 0x3B
#define NVM_PAR_P5 0x3C
#define NVM_PAR_P6 0x3E
#define NVM_PAR_P7 0x40
#define NVM_PAR_P8 0x41
#define NVM_PAR_P9 0x42
#define NVM_PAR_P10 0x44
#define NVM_PAR_P11 0x45
// ----- Values -----
#define BAROMETER_SOFTRESET 0xB6
#define BAROMETER_GPIO_PIN GPIO_PIN_13
#define BAROMETER_GPIO_PORT GPIOB


#define BAROMETER_READ_BIT 0x80u

// SPI control-bit helpers
#define BAROMETER_SPI_READ (1 << 6) 
#define BAROMETER_SPI_WRITE (~BAROMETER_SPI_READ)

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

// RW timings
#define BAROMETER_INITIALIZATION_TIMEOUT 50U
#define BAROMETER_READ_TIMEOUT 10U

#define BARO_CS_LOW() HAL_GPIO_WritePin(BAROMETER_GPIO_PORT, BAROMETER_GPIO_PIN, GPIO_PIN_RESET)
#define BARO_CS_HIGH() HAL_GPIO_WritePin(BAROMETER_GPIO_PORT, BAROMETER_GPIO_PIN, GPIO_PIN_SET)

#define SEA_LEVEL_PRESSURE 101325.0f

// ----- Types -----
typedef struct
{
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    int16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    float t_lin;
} BMP390_calib_data_t;

// Extern storage (defined in barometer.c)
extern BMP390_calib_data_t calib_data;
extern float ground_level_pressure;

// API
HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef get_temperature_pressure(SPI_HandleTypeDef *hspi, float *temperature_c, float *pressure_pa);
HAL_StatusTypeDef get_pressure(SPI_HandleTypeDef *hspi, float *pressure_pa);
HAL_StatusTypeDef get_temperature(SPI_HandleTypeDef *hspi, float *temperature_c);

// altitude helpers
float compute_relative_altitude(float pressure);
float get_relative_altitude(SPI_HandleTypeDef *hspi);
