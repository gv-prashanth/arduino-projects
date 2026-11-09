#!/usr/bin/env python3
"""Compile Arduino's original/copied-header layout without an ESP32 toolchain."""

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile


def run(command, directory):
    result = subprocess.run(command, cwd=directory, text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError("Header include regression failed:\n" + result.stdout + result.stderr)


def main():
    project = Path(__file__).resolve().parent.parent
    compiler = shlex.split(os.environ.get("CXX", "c++"))
    c_compiler = shlex.split(os.environ.get("CC", "cc"))
    with tempfile.TemporaryDirectory(prefix="gts-radio-headers-") as temporary:
        directory = Path(temporary)
        original = directory / "original sketch"
        build = directory / "arduino build"
        copied = build / "sketch"
        library = directory / "installed library"
        for path in (original, copied, library):
            path.mkdir(parents=True)
        for name in ("DisplayProfile.h", "RadioTftSetup.h"):
            # Arduino compiles a copied sketch, while -include can resolve the
            # original via its global sketch include path. These must be real
            # separate files, not links that #pragma once can identify as one.
            shutil.copyfile(project / name, original / name)
            shutil.copyfile(project / name, copied / name)
            original_time = (original / name).stat().st_mtime
            os.utime(copied / name, (original_time + 10, original_time + 10))
        shutil.copyfile(project / "build_opt.h", build / "build_opt.h")
        options = ["-I", str(original), "@" + str(build / "build_opt.h")]

        sketch = copied / "radio.ino.cpp"
        sketch.write_text('''#include "RadioTftSetup.h"
#include "DisplayProfile.h"
extern unsigned long librarySetup();
static_assert(RadioDisplay::ScreenWidth == EXPECTED_WIDTH, "Wrong display selection");
static_assert(RadioDisplay::ScreenHeight == EXPECTED_HEIGHT, "Wrong display selection");
int main() { return librarySetup() == USER_SETUP_ID ? 0 : 1; }
''')
        library_source = library / "display.cpp"
        library_source.write_text('''// Configuration must arrive through build_opt.h, not a local include.
#ifndef USER_SETUP_LOADED
#error "Display library did not receive the project configuration"
#endif
#if RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24
#ifndef ILI9341_DRIVER
#error "Shield driver missing in library compilation"
#endif
#ifdef ST7735_DRIVER
#error "Both display drivers selected"
#endif
#else
#ifndef ST7735_DRIVER
#error "ST7735 driver missing in library compilation"
#endif
#ifdef ILI9341_DRIVER
#error "Both display drivers selected"
#endif
#endif
unsigned long librarySetup() { return USER_SETUP_ID; }
''')
        for standard in ("c++11", "c++17"):
            for profile, width, height in ((1, 320, 240), (2, 160, 128)):
                executable = directory / (standard + "-" + str(profile))
                command = compiler + ["-std=" + standard, "-Wall", "-Wextra", "-Werror", "-pedantic"]
                command += options + ["-DRADIO_DISPLAY=" + str(profile),
                                      "-DEXPECTED_WIDTH=" + str(width),
                                      "-DEXPECTED_HEIGHT=" + str(height),
                                      str(sketch), str(library_source), "-o", str(executable)]
                run(command, directory)
                run([str(executable)], directory)

        # ESP32's recipes apply the same response file to C and assembly.
        c_source = directory / "plain.c"
        c_source.write_text("int radio_header_c_check(void) { return 0; }\n")
        assembly = directory / "plain.S"
        assembly.write_text(".text\n")
        for profile in (1, 2):
            selection = ["-DRADIO_DISPLAY=" + str(profile)]
            run(c_compiler + ["-std=c11", "-Werror"] + options + selection +
                ["-c", str(c_source), "-o", str(directory / ("plain-" + str(profile) + ".o"))], directory)
            run(c_compiler + ["-Werror"] + options + selection + ["-x", "assembler-with-cpp", "-c",
                str(assembly), "-o", str(directory / ("assembly-" + str(profile) + ".o"))], directory)
    print("Arduino copied-header and library configuration tests passed")


if __name__ == "__main__":
    main()
