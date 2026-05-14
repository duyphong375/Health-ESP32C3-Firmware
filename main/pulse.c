/*
 * Pulse filter and beat detection - ported from Arduino to ESP-IDF
 * Original: j.n.magee 15-10-2019
 */
#include "pulse.h"
#include <string.h>

/* ── MA Filter ── */
void ma_filter_init(ma_filter_t *f) {
    memset(f->buffer, 0, sizeof(f->buffer));
    f->nextslot = 0;
}

int16_t ma_filter_apply(ma_filter_t *f, int16_t value) {
    f->buffer[f->nextslot] = value;
    f->nextslot = (f->nextslot + 1) % NSLOT;
    int16_t total = 0;
    for (int i = 0; i < NSLOT; ++i) total += f->buffer[i];
    return total / NSLOT;
}

/* ── DC Filter ── */
void dc_filter_init(dc_filter_t *f) {
    f->sample_avg_total = 0;
}

int16_t dc_filter_apply(dc_filter_t *f, int32_t sample) {
    f->sample_avg_total += (sample - f->sample_avg_total / NSAMPLE);
    return (int16_t)(sample - f->sample_avg_total / NSAMPLE);
}

int32_t dc_filter_avg_dc(dc_filter_t *f) {
    return f->sample_avg_total / NSAMPLE;
}

/* ── Pulse ── */
void pulse_init(pulse_t *p) {
    dc_filter_init(&p->dc);
    ma_filter_init(&p->ma);
    p->cycle_max = 20;
    p->cycle_min = -20;
    p->positive  = false;
    p->prev_sig  = 0;
    p->amplitude_avg_total = 0;
}

int16_t pulse_dc_filter(pulse_t *p, int32_t sample) {
    return dc_filter_apply(&p->dc, sample);
}

int16_t pulse_ma_filter(pulse_t *p, int16_t sample) {
    return ma_filter_apply(&p->ma, sample);
}

/* Returns true when a beat (peak) is detected - identical to Arduino logic */
bool pulse_is_beat(pulse_t *p, int16_t signal) {
    bool beat = false;

    // while positive slope, record maximum
    if (p->positive && (signal > p->prev_sig))
        p->cycle_max = signal;

    // while negative slope, record minimum
    if (!p->positive && (signal < p->prev_sig))
        p->cycle_min = signal;

    // positive→negative = peak → declare beat
    if (p->positive && (signal < p->prev_sig)) {
        int amplitude = p->cycle_max - p->cycle_min;
        if (amplitude > 20 && amplitude < 3000) {
            beat = true;
            p->amplitude_avg_total += (amplitude - p->amplitude_avg_total / 4);
        }
        p->cycle_min = 0;
        p->positive  = false;
    }

    // negative→positive = valley bottom
    if (!p->positive && (signal > p->prev_sig)) {
        p->cycle_max = 0;
        p->positive  = true;
    }

    p->prev_sig = signal;
    return beat;
}

int32_t pulse_avg_dc(pulse_t *p) {
    return dc_filter_avg_dc(&p->dc);
}

int16_t pulse_avg_ac(pulse_t *p) {
    return p->amplitude_avg_total / 4;
}
