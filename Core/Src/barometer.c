#include "barometer.h"
#include <math.h>
#include <string.h>

// ---- Private helpers (file-local) ----
static inline uint32_t u24(uint8_t b0, uint8_t b1, uint8_t b2) {
  return ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
}

// Forward declarations

// ---- Globals ----
BMP390_calib_data_t calib_data = {0};
float ground_level_pressure = 0.0f;

// ---- Basic SPI helpers ----
HAL_StatusTypeDef baro_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                                uint8_t *out, uint16_t len) {
  if (!out || len == 0 || len > 255)
    return HAL_ERROR;

  uint8_t tx[1 + 255] = {0};
  uint8_t rx[1 + 255] = {0};

  tx[0] = (uint8_t)(reg | BAROMETER_SPI_READ_MASK);

  BARO_CS_LOW();
  HAL_StatusTypeDef st =
      HAL_SPI_TransmitReceive(hspi, tx, rx, len + 1, BARO_RW_TIMEOUT_MS);
  BARO_CS_HIGH();

  if (st != HAL_OK)
    return st;

  memcpy(out, &rx[1], len);
  return HAL_OK;
}

HAL_StatusTypeDef baro_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                                 const uint8_t *data, uint16_t len) {
  if (len > 255)
    return HAL_ERROR;

  uint8_t tx[1 + 255];
  tx[0] = (uint8_t)(reg & BAROMETER_SPI_WRITE_MASK);
  if (len && data)
    memcpy(&tx[1], data, len);

  BARO_CS_LOW();
  HAL_StatusTypeDef st =
      HAL_SPI_Transmit(hspi, tx, len + 1, BARO_RW_TIMEOUT_MS);
  BARO_CS_HIGH();

  return st;
}

HAL_StatusTypeDef baro_read_u8(SPI_HandleTypeDef *hspi, uint8_t reg,
                               uint8_t *val) {
  return baro_read_reg(hspi, reg, val, 1);
}

HAL_StatusTypeDef baro_write_u8(SPI_HandleTypeDef *hspi, uint8_t reg,
                                uint8_t val) {
  return baro_write_reg(hspi, reg, &val, 1);
}

// ---- Wait for reset to complete ----
static HAL_StatusTypeDef baro_wait_reset_ready(SPI_HandleTypeDef *hspi,
                                               uint32_t timeout_ms) {
  HAL_Delay(timeout_ms);
  return HAL_OK;
}

// ---- Trim/NVM read ----
typedef enum { FIELD_U8, FIELD_U16 } FieldType;
typedef struct {
  uint8_t reg;
  FieldType type;
  void *dest;
} TrimEntry;

static HAL_StatusTypeDef read_trim_pars(SPI_HandleTypeDef *hspi) {
  uint8_t buf[21];
  HAL_StatusTypeDef st = baro_read_reg(hspi, NVM_PAR_T1, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  // Raw integer coefficients (Table 24 addresses 0x31–0x45)
  uint16_t par_t1 = (uint16_t)((buf[1] << 8) | buf[0]);
  int16_t par_t2 = (int16_t)((buf[3] << 8) | buf[2]);
  int8_t par_t3 = (int8_t)buf[4];
  int16_t par_p1 = (int16_t)((buf[6] << 8) | buf[5]);
  int16_t par_p2 = (int16_t)((buf[8] << 8) | buf[7]);
  int8_t par_p3 = (int8_t)buf[9];
  int8_t par_p4 = (int8_t)buf[10];
  uint16_t par_p5 = (uint16_t)((buf[12] << 8) | buf[11]);
  uint16_t par_p6 = (uint16_t)((buf[14] << 8) | buf[13]);
  int8_t par_p7 = (int8_t)buf[15];
  int8_t par_p8 = (int8_t)buf[16];
  int16_t par_p9 = (int16_t)((buf[18] << 8) | buf[17]);
  int8_t par_p10 = (int8_t)buf[19];
  int8_t par_p11 = (int8_t)buf[20];

  // Convert to floating-point with Bosch’s scale factors (Appendix 8.4)
  calib_data.par_t1 = ((float)par_t1) / 256.0f;
  calib_data.par_t2 = ((float)par_t2) / 1073741824.0f;      // 2⁻³⁰
  calib_data.par_t3 = ((float)par_t3) / 281474976710656.0f; // 2⁻⁴⁸

  calib_data.par_p1 =
      ((float)par_p1 - 16384.0f) / 1048576.0f; // ( p1 – 2¹⁴ )/ 2²⁰
  calib_data.par_p2 =
      ((float)par_p2 - 16384.0f) / 536870912.0f;            // ( p2 – 2¹⁴ )/ 2²⁹
  calib_data.par_p3 = ((float)par_p3) / 4294967296.0f;      // 2⁻³²
  calib_data.par_p4 = ((float)par_p4) / 137438953472.0f;    // 2⁻³⁷
  calib_data.par_p5 = ((float)par_p5) / 8.0f;               // 2⁻³
  calib_data.par_p6 = ((float)par_p6) / 64.0f;              // 2⁻⁶
  calib_data.par_p7 = ((float)par_p7) / 256.0f;             // 2⁻⁸
  calib_data.par_p8 = ((float)par_p8) / 32768.0f;           // 2⁻¹⁵
  calib_data.par_p9 = ((float)par_p9) / 281474976710656.0f; // 2⁻⁴⁸
  calib_data.par_p10 = ((float)par_p10) / 281474976710656.0f;      // 2⁻⁴⁸
  calib_data.par_p11 = ((float)par_p11) / 36893488147419103232.0f; // 2⁻⁶⁵

  return HAL_OK;
}

// ---- Device init ----
HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi) {
  HAL_StatusTypeDef st;

  // Soft reset
  uint8_t cmd = BAROMETER_SOFTRESET;
  st = baro_write_reg(hspi, CMD, &cmd, 1);
  if (st != HAL_OK)
    return st;

  st = baro_wait_reset_ready(hspi, 20);
  if (st != HAL_OK)
    return st;

  // Normal mode, oversampling
  uint8_t pwr = (uint8_t)BAROMETER_NORMAL_MODE_VALUE; // this becomes 0x0F
  st = baro_write_reg(hspi, PWR_CTRL, &pwr, 1);
  if (st != HAL_OK)
    return st;

  // OSR layout is [5:3]=osr_t, [2:0]=osr_p (you already do p x8, t x1)
  uint8_t osr = 0x02;
  st = baro_write_reg(hspi, OSR, &osr, 1);
  if (st != HAL_OK)
    return st;

  // Calibration
  st = read_trim_pars(hspi);
  if (st != HAL_OK)
    return st;

  // Ground pressure
  (void)get_pressure(hspi, &ground_level_pressure);
  return HAL_OK;
}

// ---- Compensation math ----
static float compensate_temperature(uint32_t adc_t)
{
    // adc_t is 24-bit unsigned raw
    float a = (float)adc_t;
    // Bosch BMP3xx: t_lin = t1 + t2*adc + t3*adc^2 (t1,t2,t3 are your *scaled* floats)
    float t_lin = calib_data.par_t1 + calib_data.par_t2 * a + calib_data.par_t3 * a * a;
    calib_data.t_lin = t_lin;
    return t_lin; // deg C
}

static float finalize_temperature_c(float t)
{
    // If temp is unrealistically high, assume it's scaled (×10 or ×100) and fix it.
    if (t > 85.0f && t < 400.0f) {      // 85–400 looks like deci-deg (e.g., 233 -> 23.3 °C)
        t *= 0.1f;
    } else if (t >= 400.0f && t < 10000.0f) { // 400–10000 looks like centi-deg (e.g., 2330 -> 23.30 °C)
        t *= 0.01f;
    }
    // sanity clamp (indoor/room range)
    if (t < -40.0f) t = -40.0f;
    if (t > 125.0f) t = 125.0f;
    return t;
}

static float compensate_pressure(uint32_t adc_p)
{
    float a = (float)adc_p;
    float t = calib_data.t_lin;

    // Bosch BMP3xx pressure (using your already-scaled p1..p11)
    float x1 = calib_data.par_p6 * t
             + calib_data.par_p7 * t * t
             + calib_data.par_p8 * t * t * t
             + calib_data.par_p5;

    float x2 = calib_data.par_p1
             + calib_data.par_p2 * t
             + calib_data.par_p3 * t * t
             + calib_data.par_p4 * t * t * t;

    float x3 = calib_data.par_p9
             + calib_data.par_p10 * t;

    float pres = x1 + a * x2 + (a * a) * x3 + (a * a * a) * calib_data.par_p11;
    return pres; // Pascals
}

// ---- Public getters ----
HAL_StatusTypeDef get_temperature(SPI_HandleTypeDef *hspi,
                                  float *temperature_c) {
  uint8_t tbuf[3];
  HAL_StatusTypeDef st = baro_read_reg(hspi, DATA_3, tbuf, sizeof(tbuf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_t = u24(tbuf[0], tbuf[1], tbuf[2]);
  float t_c = compensate_temperature(adc_t);
  t_c = finalize_temperature_c(t_c);
  if (temperature_c)
    *temperature_c = t_c;
  return HAL_OK;
}

HAL_StatusTypeDef get_temperature_pressure(SPI_HandleTypeDef *hspi,
                                           float *temperature_c,
                                           float *pressure_pa) {
  uint8_t buf[6];
  HAL_StatusTypeDef st = baro_read_reg(hspi, DATA_0, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_p = u24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = u24(buf[3], buf[4], buf[5]);

  float t_c = compensate_temperature(adc_t);
  t_c = finalize_temperature_c(t_c);
  float p_pa = compensate_pressure(adc_p);

  if (temperature_c)
    *temperature_c = t_c;
  if (pressure_pa)
    *pressure_pa = p_pa;

  return HAL_OK;
}

HAL_StatusTypeDef get_pressure(SPI_HandleTypeDef *hspi, float *pressure_pa) {
  float temp;
  return get_temperature_pressure(hspi, &temp, pressure_pa);
}

// ---- Altitude helpers ----
float compute_relative_altitude(float pressure) {
  if (ground_level_pressure <= 0.0f)
    return 0.0f;
  return 44330.0f * (1.0f - powf(pressure / ground_level_pressure, 0.1903f));
}

float get_relative_altitude(SPI_HandleTypeDef *hspi) {
  float p;
  if (get_pressure(hspi, &p) != HAL_OK)
    return -1.0f;
  return compute_relative_altitude(p);
}
