#include "barometer.h"
#include "stm32g4xx_hal_def.h"
#include <stdint.h>

HAL_StatusTypeDef barometer_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *out_data, uint16_t out_len)
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

    BARO_CS_LOW;
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, out_len + 1, BAROMETER_READ_TIMEOUT);
    BARO_CS_HIGH;

    for (uint16_t i = 0; i < out_len; i++)
    {
        out_data[i] = rx_buf[i + 1];
    }
    return st;
}

static inline uint32_t u24(const uint8_t b0, const uint8_t b1, const uint8_t b2)
{
    return ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
}

/* Read calibration data from NVM registers (0x31-45). */

static inline HAL_StatusTypeDef read_trim_pars(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef st;
    uint8_t buf[2];

    // Lookup table for all calibration fields
    TrimEntry table[] = {
        {NVM_PAR_T1, FIELD_U16, &calib_data.par_t1},
        {NVM_PAR_T2, FIELD_U16, &calib_data.par_t2},
        {NVM_PAR_T3, FIELD_U8, &calib_data.par_t3},
        {NVM_PAR_P1, FIELD_U16, &calib_data.par_p1},
        {NVM_PAR_P2, FIELD_U16, &calib_data.par_p2},
        {NVM_PAR_P3, FIELD_U8, &calib_data.par_p3},
        {NVM_PAR_P4, FIELD_U8, &calib_data.par_p4},
        {NVM_PAR_P5, FIELD_U16, &calib_data.par_p5},
        {NVM_PAR_P6, FIELD_U16, &calib_data.par_p6},
        {NVM_PAR_P7, FIELD_U8, &calib_data.par_p7},
        {NVM_PAR_P8, FIELD_U8, &calib_data.par_p8},
        {NVM_PAR_P9, FIELD_U16, &calib_data.par_p9},
        {NVM_PAR_P10, FIELD_U8, &calib_data.par_p10},
        {NVM_PAR_P11, FIELD_U8, &calib_data.par_p11},
    };

    for (uint8_t i = 0; i < sizeof(table) / sizeof(table[0]); i++)
    {
        uint8_t len = (table[i].type == FIELD_U16) ? 2 : 1;

        st = barometer_read_reg(hspi, table[i].reg, buf, len);
        if (st != HAL_OK)
            return st;

        if (table[i].type == FIELD_U16)
        {
            *(uint16_t *)table[i].dest = (buf[0] << 8) | buf[1];
        }
        else
        {
            *(uint8_t *)table[i].dest = buf[0];
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi)
{
    // Writing is done by lowering CSB and sending pairs control bytes and register data. The control bytes consist of the SPI
    // register address (= full register address without bit 7) and the write command (bit7 = RW = ‘0’). Several pairs can be written
    // without raising CSB. The transaction is ended by a raising CSB.

    uint8_t cmd_reg = BAROMETER_SPI_WRITE & CMD;
    uint8_t osr_reg = BAROMETER_SPI_WRITE & OSR;
    uint8_t mode_buffer[6] = {cmd_reg, BAROMETER_SOFTRESET, cmd_reg, BAROMETER_NORMAL_MODE, osr_reg, PRESSURE_RES_HIGH};

    BARO_CS_LOW;

    HAL_StatusTypeDef st;
    st = HAL_SPI_Transmit(hspi, mode_buffer, (uint16_t)sizeof(mode_buffer), BAROMETER_INITIALIZATION_TIMEOUT);
    BARO_CS_HIGH;
    if (st != HAL_OK)
        return st;

    st = read_trim_pars(hspi);


    get_pressure(hspi, &ground_level_pressure);
    return st;
}

float BMP390_compensate_pressure(uint32_t uncomp_press)
{
    /* Temporary variables used for compensation */
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    /* Calibration data */
    partial_data1 = calib_data.par_p6 * calib_data.t_lin;
    partial_data2 = calib_data.par_p7 * (calib_data.t_lin * calib_data.t_lin);
    partial_data3 = calib_data.par_p8 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
    partial_out1 = calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib_data.par_p2 * calib_data.t_lin;
    partial_data2 = calib_data.par_p3 * (calib_data.t_lin * calib_data.t_lin);
    partial_data3 = calib_data.par_p4 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
    partial_out2 = (float)uncomp_press * (calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = calib_data.par_p9 + calib_data.par_p10 * calib_data.t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data.par_p11;

    return partial_out1 + partial_out2 + partial_data4;
}

float BMP390_compensate_temperature(uint32_t uncomp_temp)
{
    float partial_data1 = (float)(uncomp_temp - calib_data.par_t1);
    float partial_data2 = (float)(partial_data1 * calib_data.par_t2);

    /* Update the compensated temperature in calib structure since this is
     * needed for pressure calculation */
    calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data.par_t3;

    /* Returns compensated temperature */
    return calib_data.t_lin;
}

HAL_StatusTypeDef get_temperature(SPI_HandleTypeDef *hspi, float *temperature_c)
{
    HAL_StatusTypeDef st;

    uint8_t tbuf[3];
    //read temp data
    st = barometer_read_reg(hspi, DATA_3, tbuf, sizeof(tbuf));
    if (st != HAL_OK)
        return st;
    //convert the values into a 24 bit number in a 32 bit container.
    uint32_t adc_t = u24(tbuf[0], tbuf[1], tbuf[2]);
    float t_c = BMP390_compensate_temperature(adc_t);
    //if the pointer is not null set its values.
    if (temperature_c)
        *temperature_c = t_c;
    return HAL_OK;
}

HAL_StatusTypeDef get_pressure(SPI_HandleTypeDef *hspi, float *pressure_pa)
{
    HAL_StatusTypeDef st;

    float temp;
    //getting the pressure also relies on temperature, thus we will call get_temperature_pressure to ensure both are up to date.
    st = get_temperature_pressure(hspi, &temp, pressure_pa);
    return st;
}

HAL_StatusTypeDef get_temperature_pressure(SPI_HandleTypeDef *hspi, float *temperature_c, float *pressure_pa)
{
    HAL_StatusTypeDef st;

    uint8_t buf[6];
    //read both temp and pressure data at once
    st = barometer_read_reg(hspi, DATA_0, buf, sizeof(buf));
    if (st != HAL_OK)
        return st;
    //convert the values into a 24 bit number in a 32 bit container.
    uint32_t adc_p = u24(buf[0], buf[1], buf[2]);
    uint32_t adc_t = u24(buf[3], buf[4], buf[5]);
    //compensate the raw values with the calibration data stored on the chip;
    float t_c = BMP390_compensate_temperature(adc_t);
    float p_pa = BMP390_compensate_pressure(adc_p);  

    //if the pointers are not null set their values.
    if (temperature_c)
        *temperature_c = t_c;
    if (pressure_pa)
        *pressure_pa = p_pa;
    return HAL_OK;
}

float relative_altitude(float pressure) {
    if (ground_level_pressure <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - powf(pressure / ground_level_pressure, 0.1903f));
}

float get_relative_altitude(SPI_HandleTypeDef *hspi){
    float pressure;
    HAL_StatusTypeDef st = get_pressure(hspi, &pressure);
    if (st!= HAL_OK){
        return -1;
    }
    return compute_relative_altitude(pressure);
}