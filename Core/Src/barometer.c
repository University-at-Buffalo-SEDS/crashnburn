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
    uint8_t rx_buf[out_len + 1];

    tx_buf[0] = reg_byte;
    for (uint16_t i = 1; i <= out_len; i++)
    {
        tx_buf[i] = 0xFF;
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
static float BMP390_compensate_pressure(uint32_t uncomp_press, struct BMP390_calib_data *calib_data)
{
    // 0x30 .. 0x57
    /* Variable to store the compensated pressure */
    float comp_press;
    /* Temporary variables used for compensation */
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;
    /* Calibration data */
    partial_data1 = calib_data->par_p6 * calib_data->t_lin;
    partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
    partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
    partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = calib_data->par_p2 * calib_data->t_lin;
    partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
    partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
    partial_out2 = (float)uncomp_press *
                   (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;
    comp_press = partial_out1 + partial_out2 + partial_data4;
    return comp_press;
}

static float BMP390_compensate_temperature(uint32_t uncomp_temp, struct BMP390_calib_data *calib_data)
{
    float partial_data1;
    float partial_data2;
        partial_data1 = (float)(uncomp_temp - calib_data->par_t1);

        partial_data2 = (float)(partial_data1 * calib_data->par_t2);

        /* Update the compensated temperature in calib structure since this is
         * needed for pressure calculation */
        calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;
        /* Returns compensated temperature */
        return calib_data->t_lin;
}