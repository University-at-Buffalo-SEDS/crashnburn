#include "barometer.h"

static inline void baro_cs_low() { HAL_GPIO_WritePin(BAROMETER_GPIO_PORT, BAROMETER_GPIO_PIN, GPIO_PIN_RESET); }
static inline void baro_cs_high() { HAL_GPIO_WritePin(BAROMETER_GPIO_PORT, BAROMETER_GPIO_PIN, GPIO_PIN_SET); }

HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi)
{
    // Example: write a single config/mode byte
    uint8_t reg_byte = (1 << 7) | CMD;
    uint8_t mode_buffer[4] = {reg_byte, BAROMETER_SOFTRESET, reg_byte, BAROMETER_NORMAL_MODE};
    // set gpio pin low
    baro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, mode_buffer, (uint16_t)sizeof(mode_buffer), BAROMETER_INITIALIZATION_TIMEOUT);
    
    baro_cs_high();

    return st;
}

HAL_StatusTypeDef barometer_read_pressure(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *out_data, uint16_t out_len)
{
    // to read from a register, we need to set bit 7 to the command value then the rest (bits 1-6) to the register address.

    // Many barometer chips need the MSB of the register set to 1 for a read
    uint8_t reg_byte = (1 << 7) | reg;

    // TX buffer: first byte is the register, the rest are dummy (0xFF)
    uint8_t tx_buf[out_len + 1];
    uint8_t rx_buf[out_len + 1]; // includes echo of reg + returned data

    tx_buf[0] = reg_byte;
    for (uint16_t i = 1; i <= out_len; i++)
    {
        tx_buf[i] = 0xFF; // dummy bytes to clock data out
    }

    baro_cs_low();

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, out_len + 1, BAROMETER_READ_TIMEOUT);

    baro_cs_high();

    for (uint16_t i = 0; i < out_len; i++)
    {
        out_data[i] = rx_buf[i + 1];
    }
    return st;
}
