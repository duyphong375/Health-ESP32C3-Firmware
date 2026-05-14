/*
 * SSD1306 OLED driver for ESP-IDF 6.0.1 - ported from Arduino
 * 128x32 display, page-buffered (1 page = 128 bytes)
 * Uses new i2c_master.h API
 * Original: j.n.magee 15-10-2019
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR 0x3C
#endif

#define SSD1306_COLUMNS 128
#define SSD1306_PAGES   4    // 128x32: 4 pages of 8 rows each

typedef struct {
    i2c_master_dev_handle_t dev_handle;
    uint8_t currentPage;
    uint8_t pageBuf[SSD1306_COLUMNS];
} ssd1306_t;

void ssd1306_init(ssd1306_t *dev, i2c_master_bus_handle_t bus);
void ssd1306_command(ssd1306_t *dev, uint8_t cmd);
void ssd1306_fill(ssd1306_t *dev, uint8_t data);
void ssd1306_set_area(ssd1306_t *dev, uint8_t col, uint8_t page,
                      uint8_t col_range, uint8_t page_range);
void ssd1306_write_page(ssd1306_t *dev, uint8_t page);
bool ssd1306_in_page(ssd1306_t *dev, uint8_t y, uint8_t h);
void ssd1306_draw_pixel(ssd1306_t *dev, uint8_t x, uint8_t y);
void ssd1306_draw_hline(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t w);
void ssd1306_draw_vline(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t h);
void ssd1306_draw_char(ssd1306_t *dev, int x, int y, unsigned char c, int big);
void ssd1306_draw_str(ssd1306_t *dev, int x, int y, const char *s, int big);
void ssd1306_draw_xbmp(ssd1306_t *dev, uint8_t x, uint8_t y,
                       uint8_t w, uint8_t h, const uint8_t *bitmap);
void ssd1306_draw_rect(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void ssd1306_draw_box(ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void ssd1306_first_page(ssd1306_t *dev);
bool ssd1306_next_page(ssd1306_t *dev);
void ssd1306_off(ssd1306_t *dev);
void ssd1306_on(ssd1306_t *dev);
