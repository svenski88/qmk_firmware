// Copyright 2021-2026 svenski88
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "lighting.h"
#include <stdlib.h>
#include <string.h>

// Main-loop state only: the LED Matrix scheduler replaces the old Timer 3 ISR.
static uint8_t  key_age[LED_MATRIX_LED_COUNT];
static uint32_t last_update;
static uint32_t last_keypress;
static uint32_t ticks;
static uint8_t  rain_ticks;
static uint8_t  rows[MATRIX_ROWS];
static uint8_t  columns[MATRIX_COLS];

// Legacy angular positions, including the original tuning around Enter/Shift.
// clang-format off
static const uint8_t PROGMEM polar_angle[LED_MATRIX_LED_COUNT] = {
     97,  93,  89,  71,  68,  68,  68,  68,
    105,   0, 100,  96,  91,  84,  76,  67,
    120, 118, 116, 113, 109, 101,  86,  62,
    134, 135, 137, 139, 142, 148, 161, 187,
    147, 150, 153, 157, 162, 168, 176, 185,
    159, 163, 166, 170, 175, 180, 185, 190,
    196, 202, 207, 211, 215,   0,   0, 220,
    195, 204, 213, 219, 225, 230, 235,   0,
    216, 232, 239, 243, 245, 247, 249,   0,
     37,  24,  18,  14,  14,   0,   8,   0,
     58,  50,  42,  35,  35,   0,   0,   0,
     49,  42,  38,  35,  26,  32,   0,   0,
};
// clang-format on

void svenge_lighting_init(void) {
    memset(key_age, UINT8_MAX, sizeof(key_age));
    last_update = last_keypress = timer_read32();
    ticks                       = 0;
    rain_ticks                  = 0;
}

static void advance_clock(void) {
    uint32_t elapsed_ticks = timer_elapsed32(last_update) / 50;
    if (!elapsed_ticks) {
        return;
    }
    last_update += elapsed_ticks * 50;
    ticks += elapsed_ticks;
    for (uint8_t i = 0; i < LED_MATRIX_LED_COUNT; ++i) {
        key_age[i] = elapsed_ticks >= (uint32_t)(UINT8_MAX - key_age[i]) ? UINT8_MAX : key_age[i] + elapsed_ticks;
    }
    rain_ticks = MIN((uint32_t)UINT8_MAX, rain_ticks + elapsed_ticks);
}

void svenge_record_key(uint8_t row, uint8_t column) {
    // Advance before resetting a hit so time spent in solid mode cannot age a new hit.
    advance_clock();
    last_keypress = timer_read32();
    if (row >= MATRIX_ROWS || column >= MATRIX_COLS) {
        return;
    }
    uint8_t led = g_led_config.matrix_co[row][column];
    if (led < LED_MATRIX_LED_COUNT) {
        key_age[led] = 0;
        if (row == 5 && column == 6) {
            for (uint8_t i = 3; i <= 7; ++i) {
                key_age[i] = 0;
            }
        }
    }
}

static uint8_t faded_value(uint8_t index, uint8_t speed, bool splash) {
    uint16_t offset = key_age[index] * (splash ? 18 + speed * 9 : 3 + speed * 5);
    return offset >= led_matrix_get_val() ? 0 : led_matrix_get_val() - offset;
}

static uint8_t wave_value(uint8_t position, uint8_t phase) {
    uint8_t  distance = MIN((uint8_t)(position - phase), (uint8_t)(phase - position));
    uint16_t offset   = distance * 8;
    return offset >= led_matrix_get_val() ? 0 : led_matrix_get_val() - offset;
}

static bool is_modifier_channel(uint8_t i) {
    return i < 10 || i == 16 || i == 24 || i == 32 || (i >= 40 && i < 56) || i == 61 || i == 62 || i == 69 || i == 70 || i == 76 || i == 78 || i >= 84;
}

bool svenge_render(effect_params_t *params, enum svenge_effect effect) {
    uint8_t speed = led_matrix_get_speed() >> 6;
    if (params->iter == 0) {
        advance_clock();
        if (params->init) {
            rain_ticks = 0;
        }
        if (effect == SV_RAINDROPS && rain_ticks >= 5 - speed) {
            for (uint8_t i = 0; i < 18; ++i) {
                key_age[rand() % LED_MATRIX_LED_COUNT] = 0;
            }
            rain_ticks = 0;
        }
        if (effect == SV_ROWSPLASH || effect == SV_CROSSPLASH) {
            memset(rows, 0, sizeof(rows));
            memset(columns, 0, sizeof(columns));
            for (uint8_t row = 0; row < MATRIX_ROWS; ++row) {
                for (uint8_t col = 0; col < MATRIX_COLS; ++col) {
                    uint8_t led = g_led_config.matrix_co[row][col];
                    // NO_LED is 255 and must never index the 96-element age array.
                    if (led >= LED_MATRIX_LED_COUNT) {
                        continue;
                    }
                    uint8_t value = faded_value(led, speed, true);
                    rows[row]     = MAX(rows[row], value);
                    columns[col]  = MAX(columns[col], value);
                }
            }
        }
    }

    LED_MATRIX_USE_LIMITS(led_min, led_max);
    uint8_t phase = ticks << speed;
    for (uint8_t i = led_min; i < led_max; ++i) {
        uint8_t value = 0;
        if (HAS_ANY_FLAGS(g_led_config.flags[i], params->flags)) {
            switch (effect) {
                case SV_BREATHING: {
                    uint16_t triangle = phase < 128 ? phase * 2 : 510 - phase * 2;
                    value             = MIN(triangle, led_matrix_get_val());
                    break;
                }
                case SV_RADIAL:
                    value = wave_value(pgm_read_byte(&polar_angle[i]), phase);
                    break;
                case SV_WAVE:
                    value = wave_value(g_led_config.point[i].x, phase);
                    break;
                case SV_KEYFADE:
                case SV_RAINDROPS:
                    value = faded_value(i, speed, false);
                    break;
                case SV_ROWSPLASH:
                case SV_CROSSPLASH:
                    for (uint8_t row = 0; row < MATRIX_ROWS; ++row) {
                        for (uint8_t col = 0; col < MATRIX_COLS; ++col) {
                            if (g_led_config.matrix_co[row][col] == i) {
                                value = MAX(value, effect == SV_ROWSPLASH ? rows[row] : MAX(rows[row], columns[col]));
                            }
                        }
                    }
                    if (i >= 3 && i <= 7) {
                        value = effect == SV_ROWSPLASH ? rows[5] : MAX(rows[5], columns[6]);
                    }
                    break;
                case SV_MODS:
                    value = is_modifier_channel(i) ? led_matrix_get_val() : 0;
                    break;
            }
        }
        led_matrix_set_value(i, value);
    }
    return led_matrix_check_finished_leds(led_max);
}

static void indicator_group(uint8_t first, uint8_t count, uint8_t value) {
    for (uint8_t i = first; i < first + count; ++i) {
        led_matrix_set_value(i, value);
    }
}

bool svenge_indicators(void) {
    uint8_t mods  = get_mods() | get_weak_mods() | get_oneshot_mods();
    bool    shift = mods & MOD_MASK_SHIFT;
    // Preserve the original caps/shift XOR display on F5-F8 at full brightness.
    indicator_group(45, 4, host_keyboard_led_state().caps_lock != shift ? 255 : 0);
    if (mods & MOD_MASK_CTRL) {
        indicator_group(41, 4, 255);
    }
    if (mods & MOD_MASK_ALT) {
        indicator_group(49, 4, 255);
    }
    if (layer_state_is(3)) {
        led_matrix_set_value(62, 255);
    } else if (layer_state_is(2)) {
        led_matrix_set_value(70, 255);
    } else if (layer_state_is(1)) {
        led_matrix_set_value(78, 255);
    }

    uint32_t idle_ms = timer_elapsed32(last_keypress);
    if (idle_ms <= 100 || (idle_ms < 50000 && (idle_ms / 50) % 40 > 18)) {
        led_matrix_set_value(55, 255);
    } else if (idle_ms >= 50000) {
        led_matrix_set_value(40, 255);
    }
    // Replace the board's caps/scroll indicators with the personal display above.
    return false;
}
