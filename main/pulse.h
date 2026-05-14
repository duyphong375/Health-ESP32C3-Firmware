/*
 * Pulse filter and beat detection - ported from Arduino to ESP-IDF
 * Original: j.n.magee 15-10-2019
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define NSAMPLE 24  // EMA DC removal filter alpha = 1/NSAMPLE
#define NSLOT   4   // Moving Average Filter over NSLOT values

/* Moving Average Filter */
typedef struct {
    int16_t buffer[NSLOT];
    uint8_t nextslot;
} ma_filter_t;

/* DC Removal Filter */
typedef struct {
    int32_t sample_avg_total;
} dc_filter_t;

/* Pulse / Beat Detector */
typedef struct {
    dc_filter_t dc;
    ma_filter_t ma;
    int16_t amplitude_avg_total;
    int16_t cycle_max;
    int16_t cycle_min;
    bool    positive;
    int16_t prev_sig;
} pulse_t;

/* MA filter */
void    ma_filter_init(ma_filter_t *f);
int16_t ma_filter_apply(ma_filter_t *f, int16_t value);

/* DC filter */
void    dc_filter_init(dc_filter_t *f);
int16_t dc_filter_apply(dc_filter_t *f, int32_t sample);
int32_t dc_filter_avg_dc(dc_filter_t *f);

/* Pulse (combines DC + MA + beat detect) */
void    pulse_init(pulse_t *p);
int16_t pulse_dc_filter(pulse_t *p, int32_t sample);
int16_t pulse_ma_filter(pulse_t *p, int16_t sample);
bool    pulse_is_beat(pulse_t *p, int16_t signal);
int32_t pulse_avg_dc(pulse_t *p);
int16_t pulse_avg_ac(pulse_t *p);
