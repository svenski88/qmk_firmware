// Copyright 2026 svenski88
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "led_matrix.h"

enum svenge_effect {
    SV_BREATHING,
    SV_RADIAL,
    SV_WAVE,
    SV_KEYFADE,
    SV_ROWSPLASH,
    SV_CROSSPLASH,
    SV_MODS,
    SV_RAINDROPS,
};

void svenge_lighting_init(void);
void svenge_record_key(uint8_t row, uint8_t column);
bool svenge_render(effect_params_t *params, enum svenge_effect effect);
bool svenge_indicators(void);
