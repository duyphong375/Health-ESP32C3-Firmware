/*
 * MAX30102 driver for ESP-IDF 6.0.1 - ported from Arduino
 * Uses new i2c_master.h API (replaces Wire.h)
 * Original: j.n.magee 15-10-2019
 */
#include "max30102.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "MAX30102";
static const uint8_t MAX_30102_ID = 0x15;

/* ── I2C low-level ── */

uint8_t max30102_read_reg(max30102_t *dev, uint8_t reg) {
    uint8_t value = 0;
    esp_err_t ret = i2c_master_transmit_receive(dev->dev_handle,
                        &reg, 1, &value, 1, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return value;
}

void max30102_write_reg(max30102_t *dev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    esp_err_t ret = i2c_master_transmit(dev->dev_handle, buf, 2, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
}

/* ── Init & Setup ── */

bool max30102_init(max30102_t *dev, i2c_master_bus_handle_t bus) {
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MAX30105_ADDRESS,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg, &dev->dev_handle));
    memset(&dev->sense, 0, sizeof(dev->sense));

    uint8_t id = max30102_read_reg(dev, REG_PART_ID);
    ESP_LOGI(TAG, "Part ID: 0x%02X (expected 0x%02X)", id, MAX_30102_ID);
    return (id == MAX_30102_ID);
}

void max30102_setup(max30102_t *dev) {
    max30102_write_reg(dev, REG_MODE_CONFIG, 0x40);  // reset
    vTaskDelay(pdMS_TO_TICKS(500));
    max30102_write_reg(dev, REG_FIFO_WR_PTR, 0x00);
    max30102_write_reg(dev, REG_OVF_COUNTER, 0x00);
    max30102_write_reg(dev, REG_FIFO_RD_PTR, 0x00);
    max30102_write_reg(dev, REG_FIFO_CONFIG, 0x4f);  // avg=4, rollover=off, almostfull=17
    max30102_write_reg(dev, REG_MODE_CONFIG, 0x03);   // SpO2 mode
    max30102_write_reg(dev, REG_SPO2_CONFIG, 0x27);   // ADC=4096nA, 100Hz, 411µs
    max30102_write_reg(dev, REG_LED1_PA,     0x17);   // ~6mA IR
    max30102_write_reg(dev, REG_LED2_PA,     0x17);   // ~6mA Red
    max30102_write_reg(dev, REG_PILOT_PA,    0x1F);   // ~6mA Pilot
}

void max30102_off(max30102_t *dev) {
    max30102_write_reg(dev, REG_MODE_CONFIG, 0x80);
}

/* ── FIFO access ── */

uint8_t max30102_available(max30102_t *dev) {
    int8_t n = dev->sense.head - dev->sense.tail;
    if (n < 0) n += STORAGE_SIZE;
    return (uint8_t)n;
}

uint32_t max30102_get_red(max30102_t *dev) {
    return dev->sense.red[dev->sense.tail];
}

uint32_t max30102_get_ir(max30102_t *dev) {
    return dev->sense.IR[dev->sense.tail];
}

void max30102_next_sample(max30102_t *dev) {
    if (max30102_available(dev)) {
        dev->sense.tail++;
        dev->sense.tail %= STORAGE_SIZE;
    }
}

/* Read new samples from sensor FIFO into local circular buffer */
uint16_t max30102_check(max30102_t *dev) {
    uint8_t rdPtr = max30102_read_reg(dev, REG_FIFO_RD_PTR);
    uint8_t wrPtr = max30102_read_reg(dev, REG_FIFO_WR_PTR);
    int numberOfSamples = 0;

    if (rdPtr != wrPtr) {
        numberOfSamples = wrPtr - rdPtr;
        if (numberOfSamples < 0) numberOfSamples += 32;

        int bytesToRead = numberOfSamples * 6;  // 3 bytes IR + 3 bytes Red per sample
        if (bytesToRead > 30) bytesToRead = 30;  // cap at 5 samples (30 bytes)

        uint8_t reg = REG_FIFO_DATA;
        uint8_t fifo_buf[30];
        esp_err_t ret = i2c_master_transmit_receive(dev->dev_handle,
                            &reg, 1, fifo_buf, bytesToRead, 100);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "FIFO read failed: %s", esp_err_to_name(ret));
            return 0;
        }

        int idx = 0;
        while (idx + 6 <= bytesToRead) {  /* mỗi sample = 3 byte Red + 3 byte IR */
            dev->sense.head++;
            dev->sense.head %= STORAGE_SIZE;

            /* Red sample (3 bytes, 18-bit) - MAX30102 outputs LED1 (Red) first */
            uint32_t red_val = ((uint32_t)fifo_buf[idx]   << 16)
                             | ((uint32_t)fifo_buf[idx+1] << 8)
                             |  (uint32_t)fifo_buf[idx+2];
            dev->sense.red[dev->sense.head] = red_val & 0x3FFFF;

            /* IR sample (3 bytes, 18-bit) - MAX30102 outputs LED2 (IR) second */
            uint32_t ir_val = ((uint32_t)fifo_buf[idx+3] << 16)
                            | ((uint32_t)fifo_buf[idx+4] << 8)
                            |  (uint32_t)fifo_buf[idx+5];
            dev->sense.IR[dev->sense.head] = ir_val & 0x3FFFF;

            idx += 6;
        }
    }
    return (uint16_t)numberOfSamples;
}
