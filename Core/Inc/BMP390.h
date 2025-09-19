#pragma once
// REGISTER ADDRESSES

// General RO
#define CHIP_ID 0x00
#define REV_ID 0x01
#define ERR_REG 0x02
#define STATUS 0x03

// Pressure RO
#define DATA_0 0x04
#define DATA_1 0x05
#define DATA_2 0x06

// Temperature RO
#define DATA_3 0x07
#define DATA_4 0x08
#define DATA_5 0x09

// Sensor time (one 24 bit reading is split across 3 registers) RO
#define SENSORTIME_0 0x0C
#define SENSORTIME_1 0x0D
#define SENSORTIME_2 0x0E

// Event (sensor status flags) and interrupt status RO
#define EVENT 0x10
#define INT_STATUS 0x11

// FIFO (0x12-14 RO; 0x15-18 RW)
#define FIFO_LENGTH_0 0x12
#define FIFO_LENGTH_1 0x13
#define FIFO_DATA 0x14
#define FIFO_WTM_0 0x15
#define FIFO_WTM_1 0x16
#define FIFO_CONFIG_0 0x17
#define FIFO_CONFIG_1 0x18

// Genral interface RW
#define INT_CTRL 0x19
#define IF_CONF 0x1A
#define PWR_CTRL 0x1B
#define OSR 0x1C
#define ODR 0x1D
#define CONFIG 0x1F
#define CMD 0x7E