"""Compile the real board mapping and lighting code against a small host QMK shim."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
KEYMAP = HERE.parent / "keyboards/wilba_tech/wt75_c/keymaps/svenge"
GCC = os.environ.get("CC") or shutil.which("gcc") or "C:/UTIL/QMK_MSYS/mingw64/bin/gcc.exe"
ENV = os.environ.copy()
ENV["PATH"] = str(Path(GCC).resolve().parent) + os.pathsep + ENV["PATH"]
QMK = os.environ.get("QMK_BIN") or shutil.which("qmk") or str(ROOT / ".build/uv/qmk/Scripts/qmk.exe")
ENV["QMK_HOME"] = str(ROOT)
if os.name == "nt":
    ENV["MSYSTEM"] = "MINGW64"
    ENV["PATH"] = "C:/UTIL/QMK_MSYS/usr/bin" + os.pathsep + ENV["PATH"]

with tempfile.TemporaryDirectory(prefix="wt75-tests-") as directory:
    tmp = Path(directory)
    for name in ("quantum.h", "led_matrix.h"):
        (tmp / name).write_text('#include "fake_qmk.h"\n#include "drivers/led/issi/is31fl3736-mono.h"\n', encoding="utf-8")
    driver = tmp / "drivers/led/issi/is31fl3736-mono.h"
    driver.parent.mkdir(parents=True)
    driver.write_text('#include "fake_qmk.h"\n' + "\n".join(
        f"#define SW{row + 1}_CS{col + 1} {row * 8 + col}"
        for row in range(12) for col in range(8)
    ), encoding="utf-8")
    executable = tmp / ("test.exe" if os.name == "nt" else "test")
    generated = tmp / "default_keyboard.c"
    subprocess.run([QMK, "generate-keyboard-c", "-kb", "wilba_tech/wt75_c", "-o", str(generated)],
                   cwd=ROOT, env=ENV, check=True)
    if os.name == "nt":
        # MinGW's weak-data emulation differs from AVR/ELF. There is no override
        # in this harness, so make only the generated fixture's definition strong.
        generated.write_text(generated.read_text().replace('__attribute__ ((weak)) ', ''), encoding="utf-8")
    subprocess.run([
        GCC, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-O2",
        "-DLED_MATRIX_ENABLE", '-DQMK_KEYBOARD_H="quantum.h"',
        "-I", str(tmp), "-I", str(HERE), "-I", str(KEYMAP),
        str(ROOT / "keyboards/wilba_tech/wt75_c/wt75_c.c"),
        str(generated),
        str(KEYMAP / "lighting.c"), str(HERE / "test_lighting.c"), "-o", str(executable),
    ], check=True, env=ENV)
    subprocess.run([str(executable)], check=True, env=ENV)
