#include "barometer.h"
#include "stm32g4xx_hal_def.h"
#include "telemetry.h"
#include <math.h>
#include <string.h>

// ---- Private helpers (file-local) ----
static inline uint32_t u24(uint8_t b0, uint8_t b1, uint8_t b2) {
  return ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
}

// ---- Globals ----
BMP390_calib_data_t calib_data = {0};
float ground_level_pressure = 0.0f;

// ---- Basic SPI helpers ----
HAL_StatusTypeDef baro_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg,
                                uint8_t *out, uint16_t len) {
  if (!out || len == 0 || len > 255)
    return HAL_ERROR;

  uint8_t head[2] = {(uint8_t)(reg | BAROMETER_SPI_READ_MASK), 0x00};

  BARO_CS_LOW();
  // Phase 1: send addr + dummy
  HAL_StatusTypeDef st =
      HAL_SPI_Transmit(hspi, head, sizeof(head), BARO_RW_TIMEOUT_MS);
  if (st != HAL_OK) {
    BARO_CS_HIGH();
    return st;
  }

  // Phase 2: read payload
  st = HAL_SPI_Receive(hspi, out, len, BARO_RW_TIMEOUT_MS);
  BARO_CS_HIGH();
  return st;
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
      HAL_SPI_Transmit(hspi, tx, (uint16_t)(len + 1), BARO_RW_TIMEOUT_MS);
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

// ---- Trim/NVM read ----
static HAL_StatusTypeDef read_trim_pars(SPI_HandleTypeDef *hspi) {
  uint8_t raw[21];
  HAL_StatusTypeDef st = baro_read_reg(hspi, NVM_PAR_T1, raw, sizeof(raw));
  if (st != HAL_OK)
    return st;

  // Integer coefficients from NVM (0x31..0x45)
  uint16_t nvm_par_t1 = (uint16_t)((raw[1] << 8) | raw[0]); // U16
  int16_t nvm_par_t2 = (int16_t)((raw[3] << 8) | raw[2]);   // S16
  int8_t nvm_par_t3 = (int8_t)raw[4];

  int16_t nvm_par_p1 = (int16_t)((raw[6] << 8) | raw[5]); // S16
  int16_t nvm_par_p2 = (int16_t)((raw[8] << 8) | raw[7]); // S16
  int8_t nvm_par_p3 = (int8_t)raw[9];
  int8_t nvm_par_p4 = (int8_t)raw[10];
  uint16_t nvm_par_p5 = (uint16_t)((raw[12] << 8) | raw[11]); // U16
  uint16_t nvm_par_p6 = (uint16_t)((raw[14] << 8) | raw[13]); // U16
  int8_t nvm_par_p7 = (int8_t)raw[15];
  int8_t nvm_par_p8 = (int8_t)raw[16];
  int16_t nvm_par_p9 = (int16_t)((raw[18] << 8) | raw[17]); // S16
  int8_t nvm_par_p10 = (int8_t)raw[19];
  int8_t nvm_par_p11 = (int8_t)raw[20];

  // Convert to float
  // (Section 8.4 in the datasheet)
  calib_data.par_t1 = (float)nvm_par_t1 / powf(2.0f, -8.0f); // == nvm*256
  calib_data.par_t2 = (float)nvm_par_t2 / powf(2.0f, 30.0f);
  calib_data.par_t3 = (float)nvm_par_t3 / powf(2.0f, 48.0f);

  calib_data.par_p1 =
      ((float)nvm_par_p1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
  calib_data.par_p2 =
      ((float)nvm_par_p2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
  calib_data.par_p3 = (float)nvm_par_p3 / powf(2.0f, 32.0f);
  calib_data.par_p4 = (float)nvm_par_p4 / powf(2.0f, 37.0f);
  calib_data.par_p5 = (float)nvm_par_p5 / powf(2.0f, -3.0f); // == nvm*8
  calib_data.par_p6 = (float)nvm_par_p6 / powf(2.0f, 6.0f);
  calib_data.par_p7 = (float)nvm_par_p7 / powf(2.0f, 8.0f);
  calib_data.par_p8 = (float)nvm_par_p8 / powf(2.0f, 15.0f);
  calib_data.par_p9 = (float)nvm_par_p9 / powf(2.0f, 48.0f);
  calib_data.par_p10 = (float)nvm_par_p10 / powf(2.0f, 48.0f);
  calib_data.par_p11 = (float)nvm_par_p11 / powf(2.0f, 65.0f);

  return HAL_OK;
}

// ---- Temperature compensation — identical to Arduino/C++ (datasheet §8.5)
// ----
static float compensate_temperature(uint32_t uncomp_temp) {
  float d = (float)uncomp_temp - calib_data.par_t1;
  float t = d * calib_data.par_t2 + (d * d) * calib_data.par_t3;
  calib_data.t_lin = t;
  return t; // deg C
}

// ---- Pressure compensation — identical to Arduino/C++ (datasheet §8.6) ----
static float compensate_pressure(uint32_t uncomp_press) {
  float a = (float)uncomp_press;
  float t = calib_data.t_lin;

  float partial_out1 = calib_data.par_p5 + calib_data.par_p6 * t +
                       calib_data.par_p7 * (t * t) +
                       calib_data.par_p8 * (t * t * t);

  float partial_out2 =
      a * (calib_data.par_p1 + calib_data.par_p2 * t +
           calib_data.par_p3 * (t * t) + calib_data.par_p4 * (t * t * t));

  float partial_sq = a * a;
  float partial_cu = partial_sq * a;
  float partial_aa = partial_sq * (calib_data.par_p9 + calib_data.par_p10 * t);

  return partial_out1 + partial_out2 + partial_aa +
         partial_cu * calib_data.par_p11; // Pa
}

// --- helpers ---
static HAL_StatusTypeDef baro_wait_idle(SPI_HandleTypeDef *hspi) {
  for (int i = 0; i < 20; ++i) {
    uint8_t s = 0;
    if (baro_read_u8(hspi, STATUS, &s) != HAL_OK)
      return HAL_ERROR;
    HAL_Delay(2);
  }
  return HAL_OK;
}

static HAL_StatusTypeDef baro_try_enter_normal(SPI_HandleTypeDef *hspi) {
  uint8_t rb;

  if (baro_write_u8(hspi, PWR_CTRL, (uint8_t)PWR_CTRL_MODE_NORMAL) != HAL_OK)
    return HAL_ERROR;
  HAL_Delay(10);
  if (baro_read_u8(hspi, PWR_CTRL, &rb) != HAL_OK)
    return HAL_ERROR;
  if ((rb & PWR_CTRL_MODE_NORMAL) == PWR_CTRL_MODE_NORMAL)
    return HAL_OK;

  if ((rb & PWR_CTRL_MODE_NORMAL) == PWR_CTRL_MODE_SLEEP) {
    (void)baro_wait_idle(hspi);
    if (baro_write_u8(hspi, PWR_CTRL, (uint8_t)PWR_CTRL_MODE_NORMAL) != HAL_OK)
      return HAL_ERROR;
    HAL_Delay(10);
    if (baro_read_u8(hspi, PWR_CTRL, &rb) != HAL_OK)
      return HAL_ERROR;
    if ((rb & PWR_CTRL_MODE_NORMAL) == PWR_CTRL_MODE_NORMAL)
      return HAL_OK;
  }

  (void)baro_write_u8(hspi, PWR_CTRL, (uint8_t)0x00);
  HAL_Delay(5);
  (void)baro_write_u8(hspi, PWR_CTRL, (uint8_t)BARO_PWR_CTRL_VALUE);
  HAL_Delay(5);
  (void)baro_write_u8(hspi, PWR_CTRL, (uint8_t)PWR_CTRL_MODE_NORMAL);
  HAL_Delay(10);

  if (baro_read_u8(hspi, PWR_CTRL, &rb) != HAL_OK)
    return HAL_ERROR;
  return ((rb & 0x33) == 0x33) ? HAL_OK : HAL_ERROR;
}

// Wait for both temp+press ready (STATUS bit6|bit5)
static HAL_StatusTypeDef baro_wait_drdy(SPI_HandleTypeDef *hspi,
                                        uint32_t extra_ms) {
  // Read current ODR to compute period
  uint8_t odr_sel = 0;
  if (baro_read_u8(hspi, ODR, &odr_sel) != HAL_OK)
    return HAL_ERROR;
  odr_sel &= 0x1F; // valid range 0..18 in datasheet
  if (odr_sel > 18)
    odr_sel = 18;

  // ODR = 200 / 2^odr_sel [Hz]  ->  period_ms = 1000 / ODR = 5 * 2^odr_sel
  uint32_t period_ms = 5u << odr_sel;

  // Give ~3 frames worth of time + optional caller extra
  uint32_t timeout_ms = 3u * period_ms + extra_ms;
  if (timeout_ms < 50u)
    timeout_ms = 50u; // clamp to something reasonable

  uint32_t t0 = HAL_GetTick();
  for (;;) {
    uint8_t s = 0;
    if (baro_read_u8(hspi, STATUS, &s) != HAL_OK)
      return HAL_ERROR;

    // STATUS bits: bit6=temp_drdy, bit5=press_drdy
    if ((s & 0x60) == 0x60)
      return HAL_OK;

    if ((HAL_GetTick() - t0) >= timeout_ms)
      return HAL_TIMEOUT;

    HAL_Delay(1);
  }
}

static HAL_StatusTypeDef baro_check_drdy(SPI_HandleTypeDef *hspi,
                                        uint32_t extra_ms) {
  // Read current ODR to compute period
  uint8_t odr_sel = 0;
  if (baro_read_u8(hspi, ODR, &odr_sel) != HAL_OK)
    return HAL_ERROR;


  uint8_t s = 0;
  if (baro_read_u8(hspi, STATUS, &s) != HAL_OK)
    return HAL_ERROR;

  // STATUS bits: bit6=temp_drdy, bit5=press_drdy
  if ((s & CHIP_ID_VALUE) == CHIP_ID_VALUE)
    return HAL_OK;


  return HAL_ERROR;

  
}

// ---- Device init ----
HAL_StatusTypeDef init_barometer(SPI_HandleTypeDef *hspi) {
  HAL_StatusTypeDef st;
  uint8_t v;

  // Reset
  {
    uint8_t cmd = BAROMETER_SOFTRESET;
    st = baro_write_reg(hspi, CMD, &cmd, 1);
    if (st != HAL_OK)
      die("baro: failed to issue soft reset\r\n");
    HAL_Delay(50);
  }

  // Check ID
  st = baro_read_u8(hspi, CHIP_ID, &v);
  if (st != HAL_OK)
    die("baro: failed to read CHIP_ID\r\n");
  if (v != CHIP_ID_VALUE)
    die("baro: CHIP_ID=0x%02X (expected 0x60)\r\n", v);

  // Force 4-wire
  v = 0x00;
  (void)baro_write_reg(hspi, IF_CONF, &v, 1);
  HAL_Delay(2);

  // SLEEP -> enable sensors
  (void)baro_write_u8(hspi, PWR_CTRL, 0x00);
  HAL_Delay(2);
  if (baro_write_u8(hspi, PWR_CTRL, BARO_PWR_CTRL_VALUE) != HAL_OK)
    die("baro: enable sensors failed\r\n");
  HAL_Delay(5);
  if (baro_read_u8(hspi, PWR_CTRL, &v) != HAL_OK)
    die("baro: verify PWR_CTRL failed\r\n");
  if ((v & BARO_PWR_CTRL_VALUE) != BARO_PWR_CTRL_VALUE)
    die("baro: PWR_CTRL=0x%02X (expected 0x03)\r\n", v);

  // OSR/ODR
  if (baro_write_u8(hspi, OSR, BARO_OSR_VALUE) != HAL_OK)
    die("baro: write OSR failed\r\n"); // Tx1, Px2
  if (baro_write_u8(hspi, ODR, BARO_ODR_VALUE) != HAL_OK)
    die("baro: write ODR failed\r\n"); // 12.5Hz
  HAL_Delay(5);

  // NORMAL
  if (baro_try_enter_normal(hspi) != HAL_OK) {
    uint8_t pwr = 0, status = 0, err = 0;
    (void)baro_read_u8(hspi, PWR_CTRL, &pwr);
    (void)baro_read_u8(hspi, STATUS, &status);
    (void)baro_read_u8(hspi, ERR_REG, &err);
    die("baro: NORMAL failed, PWR_CTRL=0x%02X STATUS=0x%02X ERR=0x%02X\r\n",
        pwr, status, err);
  }

  if (read_trim_pars(hspi) != HAL_OK)
    die("baro: read calibration failed\r\n");

  uint8_t odr_sel = 0;
  (void)baro_read_u8(hspi, ODR, &odr_sel);
  odr_sel &= 0x1F;
  if (odr_sel > 18)
    odr_sel = 18;
  uint32_t period_ms = 5u << odr_sel; // ~80 ms at 12.5 Hz
  HAL_Delay(2u * period_ms);

  float p = 0.0f, t = 0.0f;
  for (int i = 0; i < 5; ++i) {
    if (get_temperature_pressure(hspi, &t, &p) != HAL_OK) {
      HAL_Delay(10);
      continue;
    }
    HAL_Delay(10);
  }

  // Sanity check baseline (reject clearly bogus values)
  if (p < 70000.0f || p > 110000.0f) {
    // Try a couple more times before giving up
    for (int i = 0; i < 5; ++i) {
      if (get_temperature_pressure(hspi, &t, &p) == HAL_OK && p >= 70000.0f &&
          p <= 110000.0f)
        break;
      HAL_Delay(10);
    }
    if (p < 70000.0f || p > 110000.0f)
      die("baro: baseline pressure invalid: %.1f Pa\r\n", p);
  }

  ground_level_pressure = p; // <-- set baseline from a valid, settled sample
  return HAL_OK;
}

void baro_rezero(SPI_HandleTypeDef *hspi) {
  float p, t;
  if (get_temperature_pressure(hspi, &t, &p) == HAL_OK && p >= 70000.0f &&
      p <= 110000.0f) {
    ground_level_pressure = p;
  }
}

// ---- Public getters ----
HAL_StatusTypeDef get_temperature(SPI_HandleTypeDef *hspi,
                                  float *temperature_c) {
  uint8_t tbuf[3];
  if (baro_wait_drdy(hspi, 5) != HAL_OK)
    return HAL_TIMEOUT;
  HAL_StatusTypeDef st = baro_read_reg(hspi, DATA_3, tbuf, sizeof(tbuf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_t = u24(tbuf[0], tbuf[1], tbuf[2]);
  float t_c = compensate_temperature(adc_t);
  if (temperature_c)
    *temperature_c = t_c;
  return HAL_OK;
}

HAL_StatusTypeDef get_temperature_pressure(SPI_HandleTypeDef *hspi,
                                           float *temperature_c,
                                           float *pressure_pa) {
  uint8_t buf[6];
  if (baro_wait_drdy(hspi, 5) != HAL_OK)
    die("baro: data not ready\r\n");
  HAL_StatusTypeDef st = baro_read_reg(hspi, DATA_0, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_p = u24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = u24(buf[3], buf[4], buf[5]);

  float t_c = compensate_temperature(adc_t);
  float p_pa = compensate_pressure(adc_p);

  if (temperature_c)
    *temperature_c = t_c;
  if (pressure_pa)
    *pressure_pa = p_pa;
  return HAL_OK;
}

HAL_StatusTypeDef get_temperature_pressure_altitude(SPI_HandleTypeDef *hspi,
                                                    float *temperature_c,
                                                    float *pressure_pa,
                                                    float *altitude_m) {
  uint8_t buf[6];
  if (baro_wait_drdy(hspi, 5) != HAL_OK)
    die("baro: data not ready\r\n");
  HAL_StatusTypeDef st = baro_read_reg(hspi, DATA_0, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_p = u24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = u24(buf[3], buf[4], buf[5]);

  float t_c = compensate_temperature(adc_t);
  float p_pa = compensate_pressure(adc_p);

  if (temperature_c)
    *temperature_c = t_c;
  if (pressure_pa)
    *pressure_pa = p_pa;
  if (altitude_m)
    *altitude_m = compute_relative_altitude(p_pa);
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

float get_relative_altitude_non_blocking(SPI_HandleTypeDef *hspi) {
  float p;
  if (get_pressure(hspi, &p) != HAL_OK)
    return -1.0f;
  return compute_relative_altitude(p);
}

static float last_temp = 0.0f;
static float last_press = 0.0f;

HAL_StatusTypeDef get_temperature_pressure_non_blocking(SPI_HandleTypeDef *hspi,
                                           float *temperature_c,
                                           float *pressure_pa) {

  if (baro_check_drdy(hspi, 5) != HAL_OK) {
    if (temperature_c)
      *temperature_c = last_temp;
    if (pressure_pa)
      *pressure_pa = last_press;
    return HAL_OK;
  }
  uint8_t buf[6];
  HAL_StatusTypeDef st = baro_read_reg(hspi, DATA_0, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_p = u24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = u24(buf[3], buf[4], buf[5]);

  float t_c = compensate_temperature(adc_t);
  float p_pa = compensate_pressure(adc_p);
  last_temp = t_c;
  last_press = p_pa;

  if (temperature_c)
    *temperature_c = t_c;
  if (pressure_pa)
    *pressure_pa = p_pa;
  return HAL_OK;
}

HAL_StatusTypeDef get_temperature_pressure_altitude_non_blocking(SPI_HandleTypeDef *hspi,
                                                    float *temperature_c,
                                                    float *pressure_pa,
                                                    float *altitude_m) {

  if (baro_check_drdy(hspi, 5) != HAL_OK) {
    if (temperature_c)
      *temperature_c = last_temp;
    if (pressure_pa)
      *pressure_pa = last_press;
    if (altitude_m)
      *altitude_m = compute_relative_altitude(last_press);
    return HAL_OK;
  }
  uint8_t buf[6];
  HAL_StatusTypeDef st = baro_read_reg(hspi, DATA_0, buf, sizeof(buf));
  if (st != HAL_OK)
    return st;

  uint32_t adc_p = u24(buf[0], buf[1], buf[2]);
  uint32_t adc_t = u24(buf[3], buf[4], buf[5]);

  float t_c = compensate_temperature(adc_t);
  float p_pa = compensate_pressure(adc_p);
  last_temp = t_c;
  last_press = p_pa;

  if (temperature_c)
    *temperature_c = t_c;
  if (pressure_pa)
    *pressure_pa = p_pa;
  if (altitude_m)
    *altitude_m = compute_relative_altitude(p_pa);
  return HAL_OK;
}

HAL_StatusTypeDef get_pressure_non_blocking(SPI_HandleTypeDef *hspi, float *pressure_pa) {
  float temp;
  return get_temperature_pressure(hspi, &temp, pressure_pa);
}