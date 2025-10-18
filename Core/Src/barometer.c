#include "barometer.h"
#include <string.h>

// ---- Private helpers (file-local) ----
static inline uint32_t u24(uint8_t b0, uint8_t b1, uint8_t b2)
{
    return ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
}

// Private forward decls
static HAL_StatusTypeDef barometer_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *out, uint16_t len);
static HAL_StatusTypeDef read_trim_pars(SPI_HandleTypeDef *hspi);
static float BMP390_compensate_pressure(uint32_t uncomp_press);
static float BMP390_compensate_temperature(uint32_t uncomp_temp);

// ---- Storage definitions (one TU only) ----
BMP390_calib_data_t calib_data = {0};
float ground_level_pressure = 0.0f;

// ---- SPI read (read = addr|0x80) ----
static HAL_StatusTypeDef barometer_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *out_data, uint16_t out_len)
{
    uint8_t tx[1] = {(uint8_t)(reg | BAROMETER_SPI_READ)};

    BARO_CS_LOW();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(hspi, tx, 1, BAROMETER_READ_TIMEOUT);
    if (st == HAL_OK)
    {
        // dummy write while reading
        memset(out_data, 0xFF, out_len);
        st = HAL_SPI_TransmitReceive(hspi, out_data, out_data, out_len, BAROMETER_READ_TIMEOUT);
    }
    BARO_CS_HIGH();
    return st;
}

// ---- Trim/NVM read (unchanged logic; uses barometer_read_reg) ----
typedef enum
{
    FIELD_U8,
    FIELD_U16
} FieldType;
typedef struct
{
    uint8_t reg;
    FieldType type;
    void *dest;
} TrimEntry;

static HAL_StatusTypeDef read_trim_pars(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef st;
    uint8_t buf[2];

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

    for (uint8_t i = 0; i < (uint8_t)(sizeof(table) / sizeof (table[0])); ++i)
    {
        uint8_t len = (table[i].type == FIELD_U16) ? 2u : 1u;
        st = barometer_read_reg(hspi, table[i].reg, buf, len);
        if (st != HAL_OK)
            return st;

        if (table[i].type == FIELD_U16)
        {
            *(uint16_t *)table[i].dest = ((uint16_t)buf[0] << 8) | buf[1];
        }
        else
        {
            *(uint8_t *)table[i].dest = buf[0];
        }
    }
    return HAL_OK;
}

// ---- Device init (write = just the address, bit7=0) ----
HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef st;

    // Soft reset
    uint8_t wr[2] = {CMD & BAROMETER_SPI_WRITE, BAROMETER_SOFTRESET};
    BARO_CS_LOW();
    st = HAL_SPI_Transmit(hspi, wr, sizeof (wr), BAROMETER_INITIALIZATION_TIMEOUT);
    BARO_CS_HIGH();
    if (st != HAL_OK)
        return st;

    HAL_Delay(2); // small reset settle

    // Set normal mode + pressure oversampling
    uint8_t cfg[] = {
        PWR_CTRL & BAROMETER_SPI_WRITE, BAROMETER_NORMAL_MODE,
        OSR & BAROMETER_SPI_WRITE, PRESSURE_RES_HIGH};
    BARO_CS_LOW();
    st = HAL_SPI_Transmit(hspi, cfg, sizeof(cfg), BAROMETER_INITIALIZATION_TIMEOUT);
    BARO_CS_HIGH();
    if (st != HAL_OK)
        return st;

    // Read calibration
    st = read_trim_pars(hspi);
    if (st != HAL_OK)
        return st;

    // Establish ground reference pressure
    (void)get_pressure(hspi, &ground_level_pressure);
    return HAL_OK;
}

// ---- Compensation (same math you had) ----
static float BMP390_compensate_pressure(uint32_t uncomp_press)
{
    float p1 = calib_data.par_p6 * calib_data.t_lin + calib_data.par_p7 * (calib_data.t_lin * calib_data.t_lin) + calib_data.par_p8 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin) + calib_data.par_p5;

    float p2 = (float)uncomp_press * (calib_data.par_p1 + calib_data.par_p2 * calib_data.t_lin + calib_data.par_p3 * (calib_data.t_lin * calib_data.t_lin) + calib_data.par_p4 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin));

    float up2 = (float)uncomp_press * (float)uncomp_press;
    float p3 = up2 * (calib_data.par_p9 + calib_data.par_p10 * calib_data.t_lin) + up2 * (float)uncomp_press * calib_data.par_p11;

    return p1 + p2 + p3;
}

static float BMP390_compensate_temperature(uint32_t uncomp_temp)
{
    float d = (float)((int32_t)uncomp_temp - (int32_t)calib_data.par_t1);
    float t = d * calib_data.par_t2 + (d * d) * calib_data.par_t3;
    calib_data.t_lin = t;
    return t;
}

// ---- Public getters ----
HAL_StatusTypeDef get_temperature(SPI_HandleTypeDef *hspi, float *temperature_c)
{
    uint8_t tbuf[3];
    HAL_StatusTypeDef st = barometer_read_reg(hspi, DATA_3, tbuf, sizeof (tbuf));
    if (st != HAL_OK)
        return st;

    uint32_t adc_t = u24(tbuf[0], tbuf[1], tbuf[2]);
    float t_c = BMP390_compensate_temperature(adc_t);
    if (temperature_c)
        *temperature_c = t_c;
    return HAL_OK;
}

HAL_StatusTypeDef get_temperature_pressure(SPI_HandleTypeDef *hspi, float *temperature_c, float *pressure_pa)
{
    uint8_t buf[6];
    HAL_StatusTypeDef st = barometer_read_reg(hspi, DATA_0, buf, sizeof (buf));
    if (st != HAL_OK)
        return st;

    uint32_t adc_p = u24(buf[0], buf[1], buf[2]);
    uint32_t adc_t = u24(buf[3], buf[4], buf[5]);

    float t_c = BMP390_compensate_temperature(adc_t);
    float p_pa = BMP390_compensate_pressure(adc_p);

    if (temperature_c)
        *temperature_c = t_c;
    if (pressure_pa)
        *pressure_pa = p_pa;
    return HAL_OK;
}

HAL_StatusTypeDef get_pressure(SPI_HandleTypeDef *hspi, float *pressure_pa)
{
    float temp; // throwaway but needed for pressure compensation
    return get_temperature_pressure(hspi, &temp, pressure_pa);
}

// ---- Altitude helpers ----
float compute_relative_altitude(float pressure)
{
    if (ground_level_pressure <= 0.0f)
        return 0.0f;
    return 44330.0f * (1.0f - powf(pressure / ground_level_pressure, 0.1903f));
}

float get_relative_altitude(SPI_HandleTypeDef *hspi)
{
    float p;
    if (get_pressure(hspi, &p) != HAL_OK)
        return -1.0f;
    return compute_relative_altitude(p);
}
