// Copyright 2021-2026 svenski88
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "lighting.h"

#include "layout.inc"

void keyboard_post_init_user(void) {
    svenge_lighting_init();
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (record->event.pressed) {
        svenge_record_key(record->event.key.row, record->event.key.col);
    }
    return true;
}

bool led_matrix_indicators_user(void) {
    return svenge_indicators();
}
