# Copyright 2026 svenski88
# SPDX-License-Identifier: GPL-2.0-or-later

# Keymap rules are loaded before post_rules.mk. Select exactly one lighting engine.
ifneq ($(strip $(LED_MATRIX_ENABLE)),yes)
    I2C_DRIVER_REQUIRED = yes
    SRC += drivers/led/issi/is31fl3736-mono.c \
           quantum/color.c \
           keyboards/wilba_tech/wt_mono_backlight.c \
           keyboards/wilba_tech/wt_main.c
endif
