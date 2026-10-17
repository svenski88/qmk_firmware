// Copyright 2026 svenski88
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MATRIX_ROWS 6
#define MATRIX_COLS 16
#define LED_MATRIX_LED_COUNT 96
#define IS31FL3736_LED_COUNT 96
#define LED_HITS_TO_REMEMBER 8
#define PROGMEM
#define pgm_read_byte(address) (*(const uint8_t *)(address))
#define NO_LED 255
#define LED_FLAG_ALL 255
#define HAS_ANY_FLAGS(bits, flags) ((bits) & (flags))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MOD_MASK_SHIFT 0x22
#define MOD_MASK_CTRL 0x11
#define MOD_MASK_ALT 0x44

typedef struct {
    uint8_t x, y;
} led_point_t;
typedef struct {
    uint8_t     matrix_co[MATRIX_ROWS][MATRIX_COLS];
    led_point_t point[LED_MATRIX_LED_COUNT];
    uint8_t     flags[LED_MATRIX_LED_COUNT];
} led_config_t;
typedef struct {
    uint8_t iter, flags;
    bool    init;
} effect_params_t;
typedef struct {
    bool caps_lock, scroll_lock;
} led_t;
typedef struct {
    uint8_t driver, v;
} is31fl3736_led_t;

extern led_config_t           g_led_config;
extern const is31fl3736_led_t g_is31fl3736_leds[96];

uint32_t timer_read32(void);
uint32_t timer_elapsed32(uint32_t since);
uint8_t  led_matrix_get_speed(void);
uint8_t  led_matrix_get_val(void);
void     led_matrix_set_value(int index, uint8_t value);
bool     led_matrix_check_finished_leds(uint8_t max);
uint8_t  get_mods(void);
uint8_t  get_weak_mods(void);
uint8_t  get_oneshot_mods(void);
led_t    host_keyboard_led_state(void);
bool     layer_state_is(uint8_t layer);
bool     led_matrix_indicators_user(void);
bool     led_matrix_indicators_kb(void);
void     keyboard_post_init_kb(void);
void     keyboard_post_init_user(void);
uint8_t  led_matrix_map_row_column_to_led_kb(uint8_t row, uint8_t col, uint8_t *leds);

#define LED_MATRIX_USE_LIMITS(min, max) \
    uint8_t min = params->iter * 20;    \
    uint8_t max = MIN((int)min + 20, LED_MATRIX_LED_COUNT)
