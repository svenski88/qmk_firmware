# WT75-C / 7V development

Personal keymaps, lighting effects, saved VIA layout, and host regression tests
now live in [svenski88/qmk_userspace](https://github.com/svenski88/qmk_userspace).
They are no longer duplicated inside this firmware checkout.

For the migrated firmware, keep this checkout on `feature/wt75-c-led-matrix` and
configure the QMK CLI to use the external userspace repository. On the owner's
Windows setup:

```sh
qmk config user.qmk_home=C:/UTIL/Code_Projects/qmk_firmware
qmk config user.overlay_dir=C:/UTIL/Code_Projects/qmk_userspace
qmk compile -kb wilba_tech/wt75_c -km svenge
```

The `master` branch mirrors official QMK `master`. It does not yet contain the
optional WT75-C LED Matrix support needed by the personal keymap. The reusable
board support originated in commit `493d943631` and can be proposed separately
against official QMK `develop` after hardware verification. This development
branch retains the migration history; personal userspace commits and this note
should stay outside that future upstream PR.

The old custom firmware is preserved on
[`legacy/wt75-c-custom`](https://github.com/svenski88/qmk_firmware/tree/legacy/wt75-c-custom)
and tagged `legacy-wt75-c-known-working`. The
[legacy release](https://github.com/svenski88/qmk_firmware/releases/tag/legacy-wt75-c-known-working)
includes the original committed HEX, its source revision, and SHA-256 checksum.
