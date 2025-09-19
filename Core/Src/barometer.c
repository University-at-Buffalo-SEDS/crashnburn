#include "barometer.h"

static inline void baro_cs_low() { HAL_GPIO_WritePin(BAROMETER_GPIO_PORT, BAROMETER_GPIO_PIN, GPIO_PIN_RESET); }
static inline void baro_cs_high() { HAL_GPIO_WritePin(BAROMETER_GPIO_PORT, BAROMETER_GPIO_PIN, GPIO_PIN_SET); }

HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi)
{
    // Writing is done by lowering CSB and sending pairs control bytes and register data. The control bytes consist of the SPI
    // register address (= full register address without bit 7) and the write command (bit7 = RW = ‘0’). Several pairs can be written
    // without raising CSB. The transaction is ended by a raising CSB.

    uint8_t cmd_reg = BAROMETER_SPI_WRITE & CMD;
    uint8_t osr_reg = BAROMETER_SPI_WRITE & OSR;
    uint8_t mode_buffer[6] = {cmd_reg, BAROMETER_SOFTRESET, cmd_reg, BAROMETER_NORMAL_MODE, osr_reg, PRESSURE_RES_HIGH};
    // set gpio pin low
    baro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, mode_buffer, (uint16_t)sizeof(mode_buffer), BAROMETER_INITIALIZATION_TIMEOUT);

    baro_cs_high();

    return st;
}

HAL_StatusTypeDef barometer_read_pressure(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *out_data, uint16_t out_len)
{
    // Reading is done by lowering CSB and first sending one control byte. The control bytes consist of the SPI register address (=
    // full register address without bit 7) and the read command (bit 7 = RW = ‘1’). After writing the control byte, one dummy byte
    // is sent and there after data bytes. The register address is automatically incremented.
    uint8_t reg_byte = BAROMETER_SPI_READ | reg;

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
