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

// Helper to write a single byte to a register
HAL_StatusTypeDef gyro_write_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)((reg << 1) | 0x00), value }; // bit0 = 0 = write
    gyro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    gyro_cs_high();
    return st;
}
// Helper to read a single register
HAL_StatusTypeDef gyro_read_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data) {
    uint8_t tx[2] = { (uint8_t)((reg << 1) | 0x01), 0x00 };  // bit0 = 1 = read
    uint8_t rx[2] = {0};
    gyro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
    gyro_cs_high();
    if (st == HAL_OK) *data = rx[1]; // second byte carries the data
    return st;
}
// Helper to read multiple bytes in one SPI transaction
HAL_StatusTypeDef gyro_read_buffer(SPI_HandleTypeDef *hspi,
                                   uint8_t start_reg,
                                   uint8_t *dst,
                                   uint16_t len)
{
    if (dst == NULL || len == 0) return HAL_ERROR;

    uint8_t cmd = (uint8_t)((start_reg << 1) | 0x01); // bit0=1 => read
    HAL_StatusTypeDef st;

    gyro_cs_low();

    // Send command (address + R/W bit)
    st = HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    if (st == HAL_OK) {
        // Read 'len' data bytes while CS stays low
        st = HAL_SPI_Receive(hspi, dst, len, HAL_MAX_DELAY);
    }

    gyro_cs_high();
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

//this uses the new helper to read multiple bytes
HAL_StatusTypeDef gyro_read(SPI_HandleTypeDef *hspi, gyro_data_t *gyro_data)
{
    uint8_t buf[6];
    HAL_StatusTypeDef st = gyro_read_buffer(hspi, RATE_X_LSB, buf, sizeof(buf));
    if (st != HAL_OK) return st;

    // LSB first, then MSB
    gyro_data->rate_x = (int16_t)((buf[1] << 8) | buf[0]);
    gyro_data->rate_y = (int16_t)((buf[3] << 8) | buf[2]);
    gyro_data->rate_z = (int16_t)((buf[5] << 8) | buf[4]);
    return HAL_OK;
}
