#include "gyro.h"

// inline functions

inline void gyro_cs_low() 
{
    HAL_GPIO_WritePin(gyro_CS_GPIO_Port, gyro_CS_Pin, GPIO_PIN_RESET);
}

inline void gyro_cs_high()
{
    HAL_GPIO_WritePin(gyro_CS_GPIO_Port, gyro_CS_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef gyro_write_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg & 0x7F, value };

    gyro_cs_low();
    HAL_SPI_Transmit(hspi, tx, 2, 2);
    gyro_cs_high();
    return HAL_OK;
}

HAL_StatusTypeDef gyro_read_register(SPI_HandleTypeDef *hspi, uint8_t reg)
{
    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2];

    gyro_cs_low();
    HAL_SPI_TransmitReceive(hspi, tx, rx, 2, 2);
    gyro_cs_high();

    return HAL_OK;
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

HAL_StatusTypeDef gyro_read(SPI_HandleTypeDef *hspi, uint16_t *gyro_data_t){

    uint8_t gyroscope_read_values[7];
    gyro_cs_low();

    uint8_t data[7] = {RATE_X_LSB | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, data, gyroscope_read_values, 7, 100);

    gyro_cs_high();

    if (status != HAL_OK) {
        return status;
    }

    gyro_data_t[0] = (uint16_t)gyroscope_read_values[2] << 8 | gyroscope_read_values[1];
    gyro_data_t[1] = (uint16_t)gyroscope_read_values[4] << 8 | gyroscope_read_values[3];
    gyro_data_t[2] = (uint16_t)gyroscope_read_values[6] << 8 | gyroscope_read_values[5];

    return HAL_OK;
}