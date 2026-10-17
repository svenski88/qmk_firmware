// Copyright 2024 QMK
// SPDX-License-Identifier: GPL-2.0-or-later

#include "quantum.h"

#if defined(MONO_BACKLIGHT_WT75_C)
#    include "drivers/led/issi/is31fl3736-mono.h"
#endif

#ifdef LED_MATRIX_ENABLE

void keyboard_post_init_kb(void) {
    // Both split-backspace switch positions share the same physical LED.
    // The data-driven mapping associates LED 61 with the right-hand switch.
    g_led_config.matrix_co[1][13] = 61;
    keyboard_post_init_user();
}

bool led_matrix_indicators_kb(void) {
    if (!led_matrix_indicators_user()) {
        return false;
    }
    if (host_keyboard_led_state().caps_lock) {
        led_matrix_set_value(16, 255);
    }
    if (host_keyboard_led_state().scroll_lock) {
        led_matrix_set_value(55, 255);
    }
    return true;
}

#endif

#if defined(LED_MATRIX_ENABLE) || defined(MONO_BACKLIGHT_WT75_C)
// clang-format off
const is31fl3736_led_t PROGMEM g_is31fl3736_leds[IS31FL3736_LED_COUNT] = {
    {0, SW1_CS1},
    {0, SW1_CS2},
    {0, SW1_CS3},
    {0, SW1_CS4},
    {0, SW1_CS5},
    {0, SW1_CS6},
    {0, SW1_CS7},
    {0, SW1_CS8},

    {0, SW2_CS1},
    {0, SW2_CS2},
    {0, SW2_CS3},
    {0, SW2_CS4},
    {0, SW2_CS5},
    {0, SW2_CS6},
    {0, SW2_CS7},
    {0, SW2_CS8},

    {0, SW3_CS1},
    {0, SW3_CS2},
    {0, SW3_CS3},
    {0, SW3_CS4},
    {0, SW3_CS5},
    {0, SW3_CS6},
    {0, SW3_CS7},
    {0, SW3_CS8},

    {0, SW4_CS1},
    {0, SW4_CS2},
    {0, SW4_CS3},
    {0, SW4_CS4},
    {0, SW4_CS5},
    {0, SW4_CS6},
    {0, SW4_CS7},
    {0, SW4_CS8},

    {0, SW5_CS1},
    {0, SW5_CS2},
    {0, SW5_CS3},
    {0, SW5_CS4},
    {0, SW5_CS5},
    {0, SW5_CS6},
    {0, SW5_CS7},
    {0, SW5_CS8},

    {0, SW6_CS1},
    {0, SW6_CS2},
    {0, SW6_CS3},
    {0, SW6_CS4},
    {0, SW6_CS5},
    {0, SW6_CS6},
    {0, SW6_CS7},
    {0, SW6_CS8},

    {0, SW7_CS1},
    {0, SW7_CS2},
    {0, SW7_CS3},
    {0, SW7_CS4},
    {0, SW7_CS5},
    {0, SW7_CS6},
    {0, SW7_CS7},
    {0, SW7_CS8},

    {0, SW8_CS1},
    {0, SW8_CS2},
    {0, SW8_CS3},
    {0, SW8_CS4},
    {0, SW8_CS5},
    {0, SW8_CS6},
    {0, SW8_CS7},
    {0, SW8_CS8},

    {0, SW9_CS1},
    {0, SW9_CS2},
    {0, SW9_CS3},
    {0, SW9_CS4},
    {0, SW9_CS5},
    {0, SW9_CS6},
    {0, SW9_CS7},
    {0, SW9_CS8},

    {0, SW10_CS1},
    {0, SW10_CS2},
    {0, SW10_CS3},
    {0, SW10_CS4},
    {0, SW10_CS5},
    {0, SW10_CS6},
    {0, SW10_CS7},
    {0, SW10_CS8},

    {0, SW11_CS1},
    {0, SW11_CS2},
    {0, SW11_CS3},
    {0, SW11_CS4},
    {0, SW11_CS5},
    {0, SW11_CS6},
    {0, SW11_CS7},
    {0, SW11_CS8},

    {0, SW12_CS1},
    {0, SW12_CS2},
    {0, SW12_CS3},
    {0, SW12_CS4},
    {0, SW12_CS5},
    {0, SW12_CS6},
    {0, SW12_CS7},
    {0, SW12_CS8}
};
// clang-format on
#endif
