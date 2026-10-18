#!/usr/bin/env python3
"""Build and run firmware logic regressions without an ESP32 or Arduino runtime."""

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sanitize", action="store_true", help="Enable AddressSanitizer and UndefinedBehaviorSanitizer")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    compiler = shlex.split(os.environ.get("CXX", "c++"))
    tests = [
        ("model", "c++11", []),
        ("input", "c++11", []),
        ("wifi_portal", "c++11", []),
        ("catalog_state", "c++11", []),
    ]
    for api in range(4):
        tests.append(("audio_service_api" + str(api), "c++17", [
            "-I", str(root / "tests" / "audio_stubs"),
            "-DRADIO_TEST_AUDIO_API=" + str(api),
            # Put the library weak default first: an accidentally weak app
            # override must fail instead of passing because it linked first.
            str(root / "tests" / "audio_stubs" / "Audio.cpp"),
            str(root / "AudioCallbacks.cpp"),
        ]))
    json_headers = root / ".pio" / "libdeps" / "esp32s3-n16r8" / "ArduinoJson" / "src"
    if (json_headers / "ArduinoJson.h").exists():
        tests.append(("network_json", "c++11", ["-I", str(json_headers)]))
    else:
        print("Network JSON tests skipped: run 'pio run' once to install pinned ArduinoJson.")
    with tempfile.TemporaryDirectory(prefix="gts-radio-tests-") as temporary:
        for name, standard, extra in tests:
            print("Checking " + name, flush=True)
            executable = str(Path(temporary) / name)
            command = compiler + ["-std=" + standard, "-Wall", "-Wextra", "-Werror", "-pedantic"]
            if args.sanitize:
                command += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
            source_name = "audio_service" if name.startswith("audio_service_api") else name
            command += extra + [str(root / "tests" / (source_name + "_test.cpp")), "-o", executable]
            subprocess.run(command, check=True, cwd=root)
            subprocess.run([executable], check=True, cwd=root)
        print("Checking unsupported PCM callback API fails explicitly", flush=True)
        unsupported = compiler + ["-std=c++17", "-DRADIO_TEST_AUDIO_API=99", "-I",
                                  str(root / "tests" / "audio_stubs"), "-c",
                                  str(root / "tests" / "audio_service_test.cpp"),
                                  "-o", str(Path(temporary) / "unsupported.o")]
        result = subprocess.run(unsupported, text=True, capture_output=True, cwd=root)
        if result.returncode == 0 or "Unsupported ESP32-audioI2S PCM callback API" not in result.stderr:
            raise RuntimeError("Unsupported PCM API did not fail with the intended diagnostic:\n" + result.stderr)
    audio_source = root / ".pio" / "libdeps" / "esp32s3-n16r8" / "ESP32-audioI2S" / "src" / "Audio.cpp"
    if audio_source.exists():
        subprocess.run([sys.executable, str(root / "tests" / "audio_patch_test.py")], check=True, cwd=root)
    else:
        print("Audio dependency patch tests skipped: run 'pio run' once to install the pinned source.")
    print("Host regression tests passed.")


if __name__ == "__main__":
    main()
