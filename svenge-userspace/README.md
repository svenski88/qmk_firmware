# Svenge's WT75-C / 7V firmware

This external QMK userspace ports the 2021 personal layout and lighting to the
optional LED Matrix support on `feature/wt75-c-led-matrix`. The board support is
kept in its own commit so a future QMK PR can exclude this directory.

## Build

From the firmware repository in a configured QMK build environment:

```sh
QMK_USERSPACE="$PWD/svenge-userspace" qmk compile -kb wilba_tech/wt75_c -km svenge
```

Or in PowerShell, with a working current QMK environment on `PATH`:

```powershell
$env:QMK_USERSPACE = (Resolve-Path ./svenge-userspace).Path
qmk compile -kb wilba_tech/wt75_c -km svenge
```

The build enables VIA, LED Matrix and LTO. The output is
`wilba_tech_wt75_c_svenge.hex`. The HEX is an ignored build artifact.
This userspace can be moved to a separate repository unchanged; it needs the
optional WT75-C support from this firmware branch until that support is upstream.

## Controls

The original four-layer layout, navigation/media keys and Fn+R bootloader shortcut
are retained. Added controls on the Fn layer:

| Shortcut | Action |
| --- | --- |
| Fn+F1 | Lighting on/off |
| Fn+F2 / Fn+F3 | Previous / next effect |
| Fn+F4 / Fn+F5 | Brightness down / up |
| Fn+F6 / Fn+F7 | Speed down / up |

Solid is the initial mode. The next eight modes are the migrated breathing,
radial wave, horizontal wave, key fade, row splash, cross splash, modifier-only,
and fading raindrops effects. The selector has no unimplemented tenth mode.
Speed uses four coarse bands of QMK's 0-255 range; standard speed keys change the
range in steps of 16, so four presses move between coarse bands. Custom modes use
the original 50 ms tick duration and up to 18 random hits per raindrop update.

The F1-F4 Ctrl, F5-F8 caps/shift XOR, and F9-F12 Alt indicators are retained,
along with layer priority 3 > 2 > 1, activity blinking on channel 55 and the idle
display after 50 seconds on channel 40. Weak and one-shot modifiers are now
included. These personal indicators override the board's caps/scroll display.

## Migration differences to review

- All rendering and hit tracking now run in the main loop. There is no custom
  timer ISR, direct register access, or interrupt-shared activity counter.
- Absent matrix positions are skipped before indexing the hit-age array.
- All five legacy spacebar channels react together. Unpopulated channels stay
  off. Physical channel population and the legacy radial tuning still need
  checking on the 7V; the old code itself marked some channels as uncertain.
- Wave wrapping now uses a consistent 256-position cycle. Scheduling comes from
  QMK rather than an exact hardware interrupt, so visual timing needs comparison.
- Turning lighting off now turns all indicators off too. While lighting is on,
  personal indicators retain their original full brightness, independent of the
  effect brightness. Activity blink visibility still depends on the base effect.
- Lighting persistence uses standard QMK LED Matrix EEPROM storage. The unused
  indicator fields and old 31-byte VIA reservation are gone. This does not migrate
  old EEPROM or VIA maps. Export the current VIA layout before flashing and
  deliberately initialize EEPROM for the new firmware.
- VIA remapping is compiled in. The old online WT75-C definition's custom lighting
  menu is not guaranteed compatible with the new LED Matrix channel or custom
  mode list. Use the Fn controls for lighting initially; a matching VIA v3
  definition and real-device verification are follow-up work.

## Validation

The legacy default, optional LED Matrix default, and personal VIA build compile
with current QMK CLI 1.2.0 and AVR GCC 8.4.0. Host regression tests exercise the
real lighting implementation and QMK-generated mapping with a small API shim:

```sh
python svenge-userspace/tests/run_tests.py
```

The test runner needs `qmk` and a host GCC on `PATH` (or `QMK_BIN` and `CC`). It
also discovers this checkout's isolated Windows tools when present. It covers all
15 absent matrix slots, invalid key coordinates, shared backspace, five-channel
spacebar tracking, all effect/speed combinations, brightness, indicator overrides,
weak/one-shot modifiers, idle behavior, and 32-bit timer wraparound.

No firmware has been flashed as part of this migration. Before using the new
firmware, review the mappings, save VIA data, and compare every LED/effect,
suspend/wake, EEPROM persistence, and typing responsiveness on the keyboard.
