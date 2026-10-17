// Copyright 2026 svenski88
// SPDX-License-Identifier: GPL-2.0-or-later

#include "fake_qmk.h"
#include "lighting.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t now;
static uint8_t  brightness = 255, speed, mods, weak_mods, oneshot_mods, layer;
static uint8_t  values[96];
static led_t    host_leds;
static bool     allow_board_indicators = true;

uint32_t timer_read32(void) {
    return now;
}
uint32_t timer_elapsed32(uint32_t since) {
    return now - since;
}
uint8_t led_matrix_get_speed(void) {
    return speed;
}
uint8_t led_matrix_get_val(void) {
    return brightness;
}
uint8_t get_mods(void) {
    return mods;
}
uint8_t get_weak_mods(void) {
    return weak_mods;
}
uint8_t get_oneshot_mods(void) {
    return oneshot_mods;
}
led_t host_keyboard_led_state(void) {
    return host_leds;
}
bool layer_state_is(uint8_t candidate) {
    return candidate == layer;
}
bool led_matrix_indicators_user(void) {
    return allow_board_indicators;
}
void keyboard_post_init_user(void) {}
bool led_matrix_check_finished_leds(uint8_t max) {
    return max < 96;
}
void led_matrix_set_value(int index, uint8_t value) {
    assert(index >= 0 && index < 96);
    values[index] = value;
}

static void render(enum svenge_effect effect) {
    effect_params_t params = {0, LED_FLAG_ALL, false};
    while (svenge_render(&params, effect)) {
        ++params.iter;
    }
}

static void assert_dark(void) {
    for (int i = 0; i < 96; ++i) {
        assert(values[i] == 0);
    }
}

int main(void) {
    keyboard_post_init_kb();
    // Check the actual board table, including split backspace and spacebar fan-out.
    unsigned holes = 0;
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 16; ++col) {
            uint8_t led = g_led_config.matrix_co[row][col];
            if (led == NO_LED) {
                ++holes;
            } else {
                assert(led < 96 && g_led_config.flags[led]);
            }
        }
    }
    assert(holes == 15);
    assert(g_led_config.matrix_co[1][13] == g_led_config.matrix_co[1][14]);
    uint8_t extra[8] = {0};
    assert(led_matrix_map_row_column_to_led_kb(5, 6, extra) == 4);
    for (int i = 0; i < 4; ++i) {
        assert(extra[i] == 3 + i);
    }
    for (int i = 0; i < 96; ++i) {
        assert(g_is31fl3736_leds[i].driver == 0 && g_is31fl3736_leds[i].v == i);
        if (g_led_config.flags[i]) {
            assert(g_led_config.point[i].x <= 224 && g_led_config.point[i].y <= 64);
        }
    }

    svenge_lighting_init();
    // All fifteen NO_LED slots must stay dark in both splash effects.
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 16; ++col) {
            if (g_led_config.matrix_co[row][col] == NO_LED) {
                svenge_record_key(row, col);
            }
        }
    }
    svenge_record_key(255, 255);
    render(SV_ROWSPLASH);
    assert_dark();
    render(SV_CROSSPLASH);
    assert_dark();

    svenge_record_key(2, 1); // Q, channel 25.
    render(SV_KEYFADE);
    assert(values[25] == 255 && values[26] == 0);
    render(SV_ROWSPLASH);
    assert(values[25] == 255 && values[69] == 255 && values[17] == 0);
    render(SV_CROSSPLASH);
    assert(values[17] == 255 && values[33] == 255 && values[18] == 0);
    now += 50;
    render(SV_KEYFADE);
    assert(values[25] == 252);
    now += 100000;
    render(SV_KEYFADE);
    assert_dark();

    svenge_record_key(5, 6);
    render(SV_KEYFADE);
    for (int i = 3; i <= 7; ++i) {
        assert(values[i] == 255);
    }

    // Every effect/speed must leave unpopulated channels off and obey brightness.
    brightness = 73;
    for (int coarse = 0; coarse < 4; ++coarse) {
        speed = coarse * 64;
        now += 300;
        for (int effect = SV_BREATHING; effect <= SV_RAINDROPS; ++effect) {
            render(effect);
            for (int i = 0; i < 96; ++i) {
                assert(values[i] <= brightness);
                if (!g_led_config.flags[i]) {
                    assert(values[i] == 0);
                }
            }
        }
    }

    memset(values, 0, sizeof(values));
    host_leds.caps_lock = true;
    assert(led_matrix_indicators_kb());
    assert(values[16] == 255);
    memset(values, 0, sizeof(values));
    allow_board_indicators = false;
    assert(!led_matrix_indicators_kb());
    assert_dark();

    assert(!svenge_indicators());
    assert(values[45] == 255 && values[48] == 255);
    weak_mods = MOD_MASK_SHIFT;
    svenge_indicators();
    assert(values[45] == 0 && values[48] == 0);
    oneshot_mods = MOD_MASK_CTRL;
    layer        = 3;
    svenge_indicators();
    assert(values[41] == 255 && values[44] == 255 && values[62] == 255);

    now = UINT32_MAX - 20;
    svenge_lighting_init();
    svenge_record_key(0, 0);
    now        = 29; // 50 ms after the hit, across timer wrap.
    brightness = 255;
    speed      = 0;
    render(SV_KEYFADE);
    assert(values[40] == 252);
    now += 50000;
    svenge_indicators();
    assert(values[40] == 255);
    puts("PASS: mapping, NO_LED safety, effects, fades, spacebar, brightness, indicators, timer wrap");
    return 0;
}
