#include "gyro.h"
#include "stm32g4xx_hal_conf.h"
#include "main.h"
#include "stm32g4xx_it.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_def.h"

void gyro_cs_low() 
{
    HAL_GPIO_WritePin(gyro_CS_GPIO_Port, gyro_CS_Pin, GPIO_PIN_RESET);
}

void gyro_cs_high()
{
    HAL_GPIO_WritePin(gyro_CS_GPIO_Port, gyro_CS_Pin, GPIO_PIN_SET);
}

void gyro_write_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg & 0x7F, value };

    gyro_cs_low();
    HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    gyro_cs_high();
}

uint8_t gyro_read_register(SPI_HandleTypeDef *hspi, uint8_t reg)
{
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;

    gyro_cs_low();
    HAL_SPI_Transmit(hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, &rx, 1, HAL_MAX_DELAY);
    gyro_cs_high();

    return rx;
}

HAL_StatusTypeDef gyro_init(SPI_HandleTypeDef *hspi)
{
    if (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
        HAL_SPI_Init(hspi);

    // read chip ID
    uint8_t id = gyro_read_register(hspi, GYRO_CHIP_ID);
    if (id != GYRO_CHIP_ID)
        return HAL_ERROR;

    // soft reset
    gyro_write_register(hspi, GYRO_SOFTRESET, 0xB6);
    HAL_Delay(50);

    // normal mode
    gyro_write_register(hspi, GYRO_LPM1, 0x00);

    // full-scale range
    gyro_write_register(hspi, GYRO_RANGE, 0x00);

    // full bandwidth
    gyro_write_register(hspi, GYRO_BANDWIDTH, 0x00);

    return HAL_OK;
}

HAL_StatusTypeDef gyro_read(uint16_t *rotational_values){

    SPI_HandleTypeDef *hspi = NULL;

    uint8_t gyroscope_read_values[6];

    // TODO: Assign the correct SPI handle to hspi before use
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, (uint8_t[]){RATE_X_LSB | 0x80}, gyroscope_read_values, 6, 100);

    if (status != HAL_OK) {
        return status;
    }

    for (int i = 0; i < 3; i++) {
        rotational_values[i] = ((uint16_t)gyroscope_read_values[2*i+1] << 8) | (uint16_t)gyroscope_read_values[2*i];
    }

    return HAL_OK;
}


/* #ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus

#include "BMI088_gyro.h"
#include "stm32g4xx_hal_conf.h"
#include "main.h"
#include "stm32g4xx_it.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_spi.c"

HAL_StatusTypeDef Gyro_Init(){
    SPI_HandleTypeDef *hspi;
    return HAL_SPI_INIT(hspi);
}

HAL_StatusTypeDef Gyro_Read(uint16_t[3] Rotational_Values) {
    
    SPI_HandleTypeDef *hspi;
    
    uint8_t[6] Gyroscope_Read_Values;

    for(i = 0; i < 6; i++){
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, RATE_X_LSB+i, Gyroscope_Read_Values+i, 7, 100); // here!

        if(status != HAL_OK){
            return status;
        }

    }

    for(i = 0; i < 3; i++){
        Rotational_Values[i] = (uint16_t)Gyroscope_Read_Values[2*i+1] << 8 | (uint16_t)Gyroscope_Read_Values[2*i];
    }

    return HAL_OK;
}

void gyro_write_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg & 0x7F, value };

    gyro_cs_low();
    HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY); // here!
    gyro_cs_high();
}

uint8_t gyro_read_register(SPI_HandleTypeDef *hspi, uint8_t reg) {
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;

    gyro_cs_low();
    HAL_SPI_Transmit(hspi, &tx, 1, HAL_MAX_DELAY); // here!
    HAL_SPI_Receive(hspi, &rx, 1, HAL_MAX_DELAY);
    gyro_cs_high();

    return rx;
}

HAL_StatusTypeDef gyro_init(SPI_HandleTypeDef *hspi) {
    if (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
        HAL_SPI_Init(hspi);

    // read chip ID
    uint8_t id = gyro_read_register(hspi, GYRO_CHIP_ID);
    if (id != GYRO_CHIP_ID)
        return HAL_ERROR;

    // soft reset
    gyro_write_register(hspi, GYRO_SOFTRESET, 0xB6);
    HAL_Delay(50);

    // normal mode
    gyro_write_register(hspi, GYRO_LPM1, 0x00);

    // full-scale range
    gyro_write_register(hspi, GYRO_RANGE, 0x00);

    // full bandwidth
    gyro_write_register(hspi, GYRO_BANDWIDTH, 0x00);

    return HAL_OK;
}

#endif
*/ 