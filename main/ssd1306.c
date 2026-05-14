/*
 * SSD1306 OLED driver for ESP-IDF 6.0.1 - ported from Arduino
 * 128x32 display, page-buffered rendering
 * Uses new i2c_master.h API (replaces Wire.h)
 * Original: j.n.magee 15-10-2019
 */
#include "ssd1306.h"
#include "font.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "SSD1306";

/* SSD1306 init sequence for 128x32 */
static const uint8_t ssd1306_config[] = {
    0xA8, 0x1F,   // Set MUX Ratio (31 for 128x32)
    0xD3, 0x00,   // Set Display Offset
    0x40,         // Set Display Start Line
    0xA1,         // Segment re-map (mirror)
    0xC8,         // COM Output Scan Direction (flip)
    0xDA, 0x02,   // COM Pins: Sequential
    0x81, 0xFF,   // Contrast: max
    0xA4,         // Output follows RAM
    0xA6,         // Normal display
    0xD5, 0x80,   // Osc Frequency
    0x8D, 0x14,   // Enable charge pump
    0xAF          // Display ON
};

/* ── I2C helpers ── */

static void ssd1306_send_commands(ssd1306_t *dev, const uint8_t *cmds, size_t len) {
    /* Control byte 0x00 = command stream */
    uint8_t buf[32];
    buf[0] = 0x00;
    if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
    memcpy(&buf[1], cmds, len);
    i2c_master_transmit(dev->dev_handle, buf, len + 1, 100);
}

static void ssd1306_send_data(ssd1306_t *dev, const uint8_t *data, size_t len) {
    /* Control byte 0x40 = data stream. Send in chunks to stay within limits */
    uint8_t buf[SSD1306_COLUMNS + 1];
    buf[0] = 0x40;
    if (len > SSD1306_COLUMNS) len = SSD1306_COLUMNS;
    memcpy(&buf[1], data, len);
    i2c_master_transmit(dev->dev_handle, buf, len + 1, 100);
}

/* ── Public API ── */

void ssd1306_command(ssd1306_t *dev, uint8_t cmd) {
    uint8_t buf[2] = { 0x00, cmd };
    i2c_master_transmit(dev->dev_handle, buf, 2, 100);
}

void ssd1306_init(ssd1306_t *dev, i2c_master_bus_handle_t bus) {
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SSD1306_I2C_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg, &dev->dev_handle));

    /* Send init sequence */
    ssd1306_send_commands(dev, ssd1306_config, sizeof(ssd1306_config));

    dev->currentPage = 0;
    memset(dev->pageBuf, 0x00, SSD1306_COLUMNS);
    ESP_LOGI(TAG, "SSD1306 128x32 initialized");
}

void ssd1306_set_area(ssd1306_t *dev, uint8_t col, uint8_t page,
                      uint8_t col_range, uint8_t page_range) {
    uint8_t cmds[] = {
        0x20, 0x01,                          // Vertical addressing mode
        0x21, col, (uint8_t)(col + col_range - 1),   // Column range
        0x22, page, (uint8_t)(page + page_range - 1)  // Page range
    };
    ssd1306_send_commands(dev, cmds, sizeof(cmds));
}

void ssd1306_fill(ssd1306_t *dev, uint8_t data) {
    ssd1306_set_area(dev, 0, 0, SSD1306_COLUMNS, SSD1306_PAGES);
    uint8_t page_data[SSD1306_COLUMNS];
    memset(page_data, data, SSD1306_COLUMNS);
    for (int p = 0; p < SSD1306_PAGES; p++) {
        ssd1306_send_data(dev, page_data, SSD1306_COLUMNS);
    }
}

void ssd1306_write_page(ssd1306_t *dev, uint8_t page) {
    ssd1306_set_area(dev, 0, page, SSD1306_COLUMNS, 1);
    ssd1306_send_data(dev, dev->pageBuf, SSD1306_COLUMNS);
}

bool ssd1306_in_page(ssd1306_t *dev, uint8_t y, uint8_t h) {
    return dev->currentPage >= y / 8 && dev->currentPage <= (y + h - 1) / 8;
}

void ssd1306_off(ssd1306_t *dev) { ssd1306_command(dev, 0xAE); }
void ssd1306_on(ssd1306_t *dev)  { ssd1306_command(dev, 0xAF); }

void ssd1306_draw_pixel(ssd1306_t *dev, uint8_t x, uint8_t y) {
    if (y / 8 != dev->currentPage) return;
    if (x >= SSD1306_COLUMNS) return;
    dev->pageBuf[x] |= 1 << (y % 8);
}

void ssd1306_draw_hline(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t w) {
    if (y / 8 != dev->currentPage) return;
    for (uint8_t i = 0; i < w; ++i) {
        if (x + i < SSD1306_COLUMNS)
            dev->pageBuf[x + i] |= 1 << (y % 8);
    }
}

void ssd1306_draw_vline(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t h) {
    for (uint8_t i = 0; i < h; ++i) ssd1306_draw_pixel(dev, x, y + i);
}

/* Double-width stretching for BIG=2 fonts */
static uint16_t stretch(uint16_t x) {
    x = (x & 0xF0) << 4 | (x & 0x0F);
    x = (x << 2 | x) & 0x3333;
    x = (x << 1 | x) & 0x5555;
    return x | x << 1;
}

void ssd1306_draw_char(ssd1306_t *dev, int x, int y, unsigned char c, int big) {
    if (c < FONT_START || c > FONT_END) return;
    for (uint8_t i = 0; i < FONT_WIDTH; i++) {
        uint16_t data = font_bitmap[c - FONT_START][i];
        if (big == 2) data = stretch(data);
        for (uint8_t d = 0; d < (uint8_t)big; d++) {
            for (uint8_t j = 0; j < FONT_HEIGHT * big; j++) {
                if (data & (1 << j))
                    ssd1306_draw_pixel(dev, x + (i * big + d), y + j);
            }
        }
    }
}

void ssd1306_draw_str(ssd1306_t *dev, int x, int y, const char *s, int big) {
    if (!ssd1306_in_page(dev, y, FONT_HEIGHT * big)) return;
    for (int k = 0; s[k] != '\0'; ++k) {
        ssd1306_draw_char(dev, x, y, s[k], big);
        x += (FONT_WIDTH + 1) * big;
    }
}

void ssd1306_draw_xbmp(ssd1306_t *dev, uint8_t x, uint8_t y,
                       uint8_t w, uint8_t h, const uint8_t *bitmap) {
    if (!ssd1306_in_page(dev, y, h)) return;
    uint8_t bytewidth = (w % 8 == 0) ? w / 8 : w / 8 + 1;
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            uint8_t bitno = i % 8;
            uint8_t data = bitmap[j * bytewidth + i / 8];
            if (data & (1 << bitno))
                ssd1306_draw_pixel(dev, x + i, y + j);
        }
    }
}

void ssd1306_draw_rect(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    ssd1306_draw_hline(dev, x, y, w);
    ssd1306_draw_hline(dev, x, y + h - 1, w);
    ssd1306_draw_vline(dev, x, y, h);
    ssd1306_draw_vline(dev, x + w - 1, y, h);
}

void ssd1306_draw_box(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    for (uint8_t j = 0; j < h; j++)
        ssd1306_draw_hline(dev, x, y + j, w);
}

void ssd1306_first_page(ssd1306_t *dev) {
    dev->currentPage = 0;
}

bool ssd1306_next_page(ssd1306_t *dev) {
    ssd1306_write_page(dev, dev->currentPage++);
    memset(dev->pageBuf, 0x00, SSD1306_COLUMNS);
    return dev->currentPage != SSD1306_PAGES;
}
