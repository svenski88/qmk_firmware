# CannonKeys Link screen development

This branch stages custom OLED work for the wired RP2040 CannonKeys Link. It
starts from official QMK `develop` at `1aa377f6c7b`, independently of the personal
WT75-C firmware branch. No keyboard-level firmware changes are needed yet.

The matching personal keymap is on `feature/link-oled` in
[svenski88/qmk_userspace](https://github.com/svenski88/qmk_userspace/tree/feature/link-oled).
Keep both checkouts on `feature/link-oled` while working on these screens. With
QMK already configured to use the external userspace:

```sh
qmk compile -kb cannonkeys/link -km svenge
qmk lint --strict -kb cannonkeys/link -km svenge
```

The baseline enables VIA, uses the stock starting layout and supplies pass-through
OLED callbacks. The existing status display, QMK logo, rotations and encoder
actions remain the baseline. The artifact is `cannonkeys_link_svenge.uf2` in the
external userspace directory; nothing is flashed automatically.

Relevant firmware sources:

- `keyboards/cannonkeys/link/link.c`: existing OLED and encoder callbacks.
- `keyboards/cannonkeys/link/config.h`: display geometry and transport pins.
- `keyboards/cannonkeys/link/keyboard.json`: split, matrix and hardware metadata.
- `drivers/oled/oled_driver.c`: calls the keyboard initializer and then the user
  initializer; no rotation-hook patch is necessary.

Custom rendering belongs in userspace
`keyboards/cannonkeys/link/keymaps/svenge/oled.c`. Return false from
`oled_task_user()` after drawing to replace the keyboard's stock content. The
userspace guide describes master/secondary versus physical left/right decisions.

For WT75-C work, use firmware `feature/wt75-c-led-matrix` and userspace `main`.
Those branches and the released 2026 firmware are preserved separately.

The baseline UF2 build and strict keymap lint passed using QMK CLI 1.2.0 and
ARM GCC 10.1.0. OLED hardware behavior has not been tested on the keyboard, and
no firmware was flashed during this setup.

## Custom screens implemented in userspace

The userspace branch now replaces the stock screens with mirrored physical-half
keypress maps, large Ctrl/Caps-XOR/Alt panels on the physical left, and the actual
highest active layer on the physical right. The saved Link VIA export supplies
all six layers, including its SpaceFN layer-tap keys.

Right encoder push [9,5] is reserved for Dashboard → Matrix rain → Off on both
screens. Dashboard enters rain at 60 seconds idle and wakes on activity; manual
rain stays selected until its deadline or a mode press. Manual Off stays off
until the next mode press. Held keys
prevent automatic idle entry. A validated, throttled custom split RPC carries
the mode, modifiers, Caps Lock, active layer and physical-key visualization to
the secondary half, independently of which side has USB.

Rain runs at minimum OLED brightness (0); the dashboard restores the configured
normal brightness. Both automatic and manual rain power down after ten minutes
of continuous rain, regardless of activity during manual rain. A fresh keypress
wakes the dashboard after this timeout. Releases, held keys and encoder rotation
leave timeout-off asleep. Manually selected Off still requires a mode press.

See the [current userspace guide](https://github.com/svenski88/qmk_userspace/blob/feature/link-oled/keyboards/cannonkeys/link/keymaps/svenge/README.md)
for previews, saved-layout import, host tests, and matching-UF2 instructions for
both halves. The new UF2 build, strict lint, and model/renderer/adapter host tests
pass. No keyboard-level code changes or automated flashing were needed.
