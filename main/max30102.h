/*
 * MAX30102 driver for ESP-IDF 6.0.1 - ported from Arduino
 * Uses new i2c_master.h API
 * Original: j.n.magee 15-10-2019
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#define MAX30105_ADDRESS    0x57

/* Register addresses */
#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D
#define REG_PILOT_PA        0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR       0x1F
#define REG_TEMP_FRAC       0x20
#define REG_TEMP_CONFIG     0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID          0xFE
#define REG_PART_ID         0xFF

#define STORAGE_SIZE 32

typedef struct {
    i2c_master_dev_handle_t dev_handle;
    struct {
        uint32_t red[STORAGE_SIZE];
        uint32_t IR[STORAGE_SIZE];
        uint8_t  head;
        uint8_t  tail;
    } sense;
} max30102_t;

bool     max30102_init(max30102_t *dev, i2c_master_bus_handle_t bus);
void     max30102_setup(max30102_t *dev);
void     max30102_off(max30102_t *dev);
uint16_t max30102_check(max30102_t *dev);
uint8_t  max30102_available(max30102_t *dev);
void     max30102_next_sample(max30102_t *dev);
uint32_t max30102_get_red(max30102_t *dev);
uint32_t max30102_get_ir(max30102_t *dev);
uint8_t  max30102_read_reg(max30102_t *dev, uint8_t reg);
void     max30102_write_reg(max30102_t *dev, uint8_t reg, uint8_t value);
