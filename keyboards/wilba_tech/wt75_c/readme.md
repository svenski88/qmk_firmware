# wilba.tech WT75-C

WT75-C is a keyboard PCB supporting 75% layout with offset arrow keys. [More info at wilba.tech](https://wilba.tech/)

* Keyboard Maintainer: [Wilba](https://github.com/wilba)
* Hardware Supported: wilba.tech WT75-C
* Hardware Availability: Custom keyboard group buys

Make example for this keyboard (after setting up your build environment):

    make wilba_tech/wt75_c:default

## Optional LED Matrix

The original Wilba mono backlight remains the default. To use QMK LED Matrix instead,
set `LED_MATRIX_ENABLE = yes` in your keymap's `rules.mk`, or build with:

    make wilba_tech/wt75_c:default LED_MATRIX_ENABLE=yes

Enable the desired [LED Matrix effects](https://docs.qmk.fm/features/led_matrix) in
your keymap's `config.h`. Standard `LM_*` keycodes control mode, brightness and speed.
The stock keymap has no lighting controls. Caps Lock and Scroll Lock keep their
board indicators; `led_matrix_indicators_user()` may return `false` to override them.

The matrix and position mapping retains the legacy 96-channel numbering, including
shared split-backspace illumination and the spacebar's extra channels. Unpopulated
channels are flagged off. Physical positions and the spacebar channels need hardware
verification before this optional mapping is considered ready for upstream submission.

LED Matrix builds exclude the custom mono engine and its timer. They use QMK's LED
Matrix EEPROM settings rather than the mono configuration. Existing firmware's
lighting settings and VIA keymaps are not migrated automatically. Export any VIA
keymap before switching firmware, and initialize EEPROM for the new build.

See the [build environment setup](https://docs.qmk.fm/#/getting_started_build_tools) and the [make instructions](https://docs.qmk.fm/#/getting_started_make_guide) for more information. Brand new to QMK? Start with our [Complete Newbs Guide](https://docs.qmk.fm/#/newbs).
