#include "gyro.h"

// inline functions

static void gyro_cs_low() 
{
    HAL_GPIO_WritePin(gyro_CS_GPIO_Port, gyro_CS_Pin, GPIO_PIN_RESET);
}

static void gyro_cs_high()
{
    HAL_GPIO_WritePin(gyro_CS_GPIO_Port, gyro_CS_Pin, GPIO_PIN_SET);
}

// HAL_StatusTypeDef gyro_write_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value)
// {
//     uint8_t tx[2] = { reg & 0x7F, value };

//     gyro_cs_low();
//     HAL_SPI_Transmit(hspi, tx, 2, 2);
//     gyro_cs_high();
//     return HAL_OK;
// }

// HAL_StatusTypeDef gyro_read_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data)
// {
//     HAL_StatusTypeDef st;
//     uint8_t tx[2] = {reg | 0x80, 0x00};
//     uint8_t rx[2];

//     gyro_cs_low();
//     st = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, 2);
//     if (st != HAL_OK)
//     {
//         return st;
//     }
//     gyro_cs_high();

//     *data = rx[1];

//     return st;
// }

/* These functions should hopefully correctly structure the messages */
HAL_StatusTypeDef gyro_write_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)((reg << 1) | 0x00), value }; // bit0 = 0 = write
    gyro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    gyro_cs_high();
    return st;
}

HAL_StatusTypeDef gyro_read_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data) {
    uint8_t tx[2] = { (uint8_t)((reg << 1) | 0x01), 0x00 };  // bit0 = 1 = read
    uint8_t rx[2] = {0};
    gyro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
    gyro_cs_high();
    if (st == HAL_OK) *data = rx[1]; // second byte carries the data
    return st;
}

HAL_StatusTypeDef gyro_init(SPI_HandleTypeDef *hspi)
{
    if (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
    {
        return HAL_BUSY;
    }

    // read chip ID
    uint8_t id;
    gyro_read_register(hspi, GYRO_CHIP_ID, &id);
    if (id != GYRO_CHIP_ID_VALUE) // check the register value not the register address.
    {
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

HAL_StatusTypeDef gyro_read(SPI_HandleTypeDef *hspi, gyro_data_t *gyro_data){

    uint8_t gyroscope_read_values[7];
    gyro_cs_low();

    uint8_t data[7] = {RATE_X_LSB | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, data, gyroscope_read_values, 7, 50);

    gyro_cs_high();

    if (status != HAL_OK) {
        return status;
    }

    gyro_data->rate_x = (uint16_t)gyroscope_read_values[2] << 8 | gyroscope_read_values[1];
    gyro_data->rate_y = (uint16_t)gyroscope_read_values[4] << 8 | gyroscope_read_values[3];
    gyro_data->rate_z = (uint16_t)gyroscope_read_values[6] << 8 | gyroscope_read_values[5];

    return HAL_OK;
}