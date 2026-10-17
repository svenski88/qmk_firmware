// Copyright 2026 svenski88
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

// Preserve the original solid-on startup and four coarse effect speeds.
#define LED_MATRIX_DEFAULT_MODE LED_MATRIX_SOLID
#define LED_MATRIX_DEFAULT_SPD 0
#define LED_MATRIX_LED_FLUSH_LIMIT 50

// These enable optional built-in alternatives after the migrated custom effects.
// Keep them disabled initially to preserve the original mode cycle and save flash.
// #define ENABLE_LED_MATRIX_SOLID_REACTIVE_SIMPLE
// #define ENABLE_LED_MATRIX_SOLID_REACTIVE_MULTICROSS
