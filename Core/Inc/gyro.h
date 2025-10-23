#include "stm32g4xx_hal.h" 
#include "stm32g4xx_hal_spi.h"
#include "main.h"
#include <stdbool.h>

/* BMI088 Gyro Register Addresses*/
#define FIFO_DATA 0x3F
#define FIFO_CONFIG_1 0x3E
#define FIFO_CONFIG_0 0x3D
#define GYRO_SELF_TEST 0x3C
#define FIFO_EXT_INT_S 0x34
#define FIFO_WM_EN 0x1E
#define INT3_INT4_IO_MAP 0x18
#define INT3_INT4_IO_CONF 0x16
#define GYRO_INT_CTRL 0x15
#define GYRO_SOFTRESET 0x14
#define GYRO_LPM1 0x11
#define GYRO_BANDWIDTH 0x10
#define GYRO_RANGE 0x0F
#define FIFO_STATUS 0x0E
#define GYRO_INT_STAT_1 0x0A
#define RATE_Z_MSB 0x07
#define RATE_Z_LSB 0x06
#define RATE_Y_MSB 0x05
#define RATE_Y_LSB 0x04
#define RATE_X_MSB 0x03
#define RATE_X_LSB 0x02
#define GYRO_CHIP_ID 0x00

HAL_StatusTypeDef gyro_write_register(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value); // return HAL status
HAL_StatusTypeDef gyro_read_register(SPI_HandleTypeDef *hspi, uint8_t reg);
HAL_StatusTypeDef gyro_init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef gyro_read(SPI_HandleTypeDef *hspi, uint16_t *gyro_data_t);

typedef enum {
    GYRO_RANGE_2000DPS = 0x00,
    GYRO_RANGE_1000DPS = 0x01,
    GYRO_RANGE_500DPS = 0x02,
    GYRO_RANGE_250DPS = 0x03,
    GYRO_RANGE_125DPS = 0x04
} GyroRange;

typedef enum {
    GYRO_BANDWIDTH_523HZ_ODR_2000HZ = 0x00,
    GYRO_BANDWIDTH_230HZ_ODR_2000HZ = 0x01,
    GYRO_BANDWIDTH_116HZ_ODR_1000HZ = 0x02,
    GYRO_BANDWIDTH_47HZ_ODR_400HZ = 0x03,
    GYRO_BANDWIDTH_23HZ_ODR_200HZ = 0x04,
    GYRO_BANDWIDTH_12HZ_ODR_100HZ = 0x05,
    GYRO_BANDWIDTH_64HZ_ODR_200HZ = 0x06,
    GYRO_BANDWIDTH_32HZ_ODR_100HZ = 0x07
} GyroBandwidth;

typedef enum {
    GYRO_NORMAL_MODE = 0x00,
    GYRO_DEEP_SUSPEND_MODE = 0x20, 
    GYRO_SUSPEND_MODE = 0x80
} GyroPowerMode;

typedef enum {
    INT4_MODE_PUSH_PULL = 0,
    INT4_MODE_OPEN_DRAIN = 1,
    INT4_MODE_ACTIVE_HIGH = 0,
    INT4_MODE_ACTIVE_LOW = 1,
    INT3_MODE_PUSH_PULL = 0,
    INT3_MODE_OPEN_DRAIN = 1,
    INT3_MODE_ACTIVE_HIGH = 0,
    INT3_MODE_ACTIVE_LOW = 1
} GyroIntPinMode;

typedef struct { 
    uint16_t rate_x;
    uint16_t rate_y;
    uint16_t rate_z;
} gyro_data_t;

typedef struct { 
    uint8_t chip_id;
    GyroRange range;
    GyroBandwidth bandwidth;
    GyroPowerMode power_mode;
    float scale_factor;
    bool (*read)(uint8_t reg, uint8_t *data, uint16_t len);
    bool (*write)(uint8_t reg, uint8_t *data, uint16_t len);
} gyro_device_t;

typedef struct {
    /* Holds output enable setting of INT pin (1 - output) or (0 - input)*/
    uint8_t int_pin_io;
    /* Holds active high/low setting of INT pin (1 - active high) or (0 - active low)*/
    uint8_t int_latch;
} gyro_int_pin_conf; 
