#!/usr/bin/env python3
"""Regenerate checked-in ASCII GFX fonts; firmware builds never run this tool."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import re
import shlex
import subprocess
import tarfile
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parent.parent
ARCHIVE_URL = "https://github.com/liberationfonts/liberation-fonts/files/7261482/liberation-fonts-ttf-2.1.5.tar.gz"
ARCHIVE_SHA256 = "7191c669bf38899f73a2094ed00f7b800553364f90e2637010a69c0e268f25d0"
SOURCE_HASHES = {
    "LiberationSans-Regular.ttf": "76d04c18ea243f426b7de1f3ad208e927008f961dc5945e5aad352d0dfde8ee8",
    "LiberationSans-Bold.ttf": "788abee4c806d660e8aee46689dd8540cd4bb98da03dcc9d171ce3efd99a9173",
    "LICENSE": "93fed46019c38bbe566b479d22148e2e8a1e85ada614accb0211c37b2c61c19b",
}
CONVERTER_SHA256 = "42e70906d68e4a7bced1c47a7bb12d2a86c29484eeb564f76449ea7a9b6c3ba1"
FONTS = [("Body", "LiberationSans-Regular.ttf", 7),
         ("Title", "LiberationSans-Bold.ttf", 10),
         ("Small", "LiberationSans-Regular.ttf", 6)]
SAMPLES = ["GTS RADIO", "Vol21", "Muted", "Akashvani / IN", "Radio Browser / IN",
           "Click:mute Hold:menu", "Click mute | Hold menu", "Playing - ready",
           "AAC 67 kbps", "Akashvani p14/14"]
GLYPH_PATTERN = r"\{\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\}"


def verified(data, expected, label):
    if hashlib.sha256(data).hexdigest() != expected:
        raise RuntimeError("Unexpected SHA-256 for " + label)
    return data


def source_directory(args, temporary):
    if args.font_dir:
        return args.font_dir
    data = args.archive.read_bytes() if args.archive else urllib.request.urlopen(ARCHIVE_URL, timeout=60).read()
    verified(data, ARCHIVE_SHA256, "Liberation 2.1.5 archive")
    directory = temporary / "source"
    directory.mkdir()
    # Extract only known regular files; never trust archive paths or symlinks.
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
        for name, digest in SOURCE_HASHES.items():
            member = archive.getmember("liberation-fonts-ttf-2.1.5/" + name)
            if not member.isfile():
                raise RuntimeError("Expected a regular font source file")
            (directory / name).write_bytes(verified(archive.extractfile(member).read(), digest, name))
    return directory


def publish(path, text, check):
    if check:
        if not path.exists() or path.read_text() != text:
            raise RuntimeError("Regenerated asset differs: " + str(path))
    else:
        path.write_text(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group(required=True)
    sources.add_argument("--font-dir", type=Path, help="Verified upstream Regular/Bold TTFs and LICENSE")
    sources.add_argument("--archive", type=Path, help="Existing pinned upstream TTF archive")
    sources.add_argument("--download", action="store_true", help="Download the SHA-256-verified pinned archive")
    parser.add_argument("--gfx-dir", type=Path, default=ROOT / ".pio/libdeps/esp32s3-n16r8/Adafruit GFX Library")
    parser.add_argument("--check", action="store_true", help="Compare generated assets without changing files")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="radio-fonts-") as scratch:
        temporary = Path(scratch)
        directory = source_directory(args, temporary)
        for name, digest in SOURCE_HASHES.items():
            verified((directory / name).read_bytes(), digest, name)
        converter_source = args.gfx_dir / "fontconvert/fontconvert.c"
        verified(converter_source.read_bytes(), CONVERTER_SHA256, "Adafruit GFX 1.12.6 fontconvert.c")
        flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs", "freetype2"], text=True))
        converter = temporary / "fontconvert"
        subprocess.run(["cc", str(converter_source), "-o", str(converter)] + flags, check=True)
        metrics = {}
        for role, filename, points in FONTS:
            converted = subprocess.check_output([str(converter), str(directory / filename), str(points), "32", "126"], text=True)
            original_name = re.search(r"const GFXfont (\w+) PROGMEM", converted).group(1)
            name = "RadioSans" + role
            converted = converted.replace(original_name, name)
            glyphs = [list(map(int, entry)) for entry in re.findall(GLYPH_PATTERN, converted)]
            if len(glyphs) != 95:
                raise RuntimeError("Expected printable ASCII coverage")
            bitmap = converted.split("Bitmaps[] PROGMEM = {", 1)[1].split("};", 1)[0]
            bitmap_bytes = len(re.findall(r"0x[0-9A-Fa-f]{2}", bitmap))
            if any(offset + (width * height + 7) // 8 > bitmap_bytes
                   for offset, width, height, advance, x, y in glyphs):
                raise RuntimeError("Glyph bitmap is outside its asset")
            top = min(row[5] for row in glyphs if row[1] and row[2])
            bottom = max(row[5] + row[2] - 1 for row in glyphs if row[1] and row[2])
            metrics[role] = {
                "source": filename, "points": points, "dpi": 141,
                "first": 32, "last": 126, "top": top, "bottom": bottom,
                "height": bottom - top + 1,
                "yAdvance": int(re.search(r"0x20, 0x7E, (\d+)", converted).group(1)),
                "bitmapBytes": bitmap_bytes,
                "sampleAdvances": {text: sum(glyphs[ord(char) - 32][3] for char in text) for text in SAMPLES},
            }
            preamble = ("#pragma once\n\n#include <Adafruit_GFX.h>\n\n"
                        "// Generated by scripts/generate_radio_fonts.py from Liberation Sans 2.1.5.\n"
                        "// Derived bitmap font renamed RadioSans; SIL OFL 1.1, see LICENSE-OFL.txt.\n"
                        "// ASCII 32-126, " + str(points) + "pt at 141 DPI. Do not edit by hand.\n\n")
            publish(ROOT / "fonts" / (name + ".h"), preamble + converted, args.check)
        license_text = "\n".join(line.rstrip() for line in (directory / "LICENSE").read_text().splitlines()).rstrip() + "\n"
        publish(ROOT / "fonts/LICENSE-OFL.txt", license_text, args.check)
        publish(ROOT / "fonts/metrics.json", json.dumps(metrics, indent=2) + "\n", args.check)
        print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
