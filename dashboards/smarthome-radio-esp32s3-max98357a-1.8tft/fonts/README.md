# RadioSans embedded fonts

These checked-in Adafruit GFX bitmap headers require no font-generation tools or
additional libraries when building the firmware in Arduino IDE or PlatformIO.
They contain printable ASCII 32–126. The application retains its existing UTF-8
transliteration before drawing text.

RadioSans is an ASCII bitmap conversion of **Liberation Sans 2.1.5**, renamed for
this modified format. The original copyright notices and the full upstream SIL
Open Font License 1.1 are included in `LICENSE-OFL.txt`. The font assets remain
under that license. The upstream reserved names are not used as the derived
fonts' primary names.

Only trailing whitespace is normalized in the bundled license text.

| Role | Source | Size at 141 DPI | Ink above baseline | Ink at/below baseline | Maximum height | Line advance |
|---|---|---:|---:|---:|---:|---:|
| Body | Regular | 7 pt | 10 px | 4 px | 14 px | 16 px |
| Title | Bold | 10 pt | 14 px | 5 px | 19 px | 23 px |
| Small | Regular | 6 pt | 9 px | 4 px | 13 px | 14 px |
| Chrome (header, filter strip, controls) | Regular | 5 pt | 7 px | 3 px | 10 px | 11 px |

The ink bounds include every printable ASCII glyph, including braces and
underscores. Individual phrases can be shorter. Use measured pixel bounds, not a
fixed character count. `metrics.json` records glyph coverage, bounds, bitmap
sizes, and representative phrase advances. `RadioFonts.h` exposes the font
pointers and ascent/descent constants.

## Source and reproduction

Official upstream release:
<https://github.com/liberationfonts/liberation-fonts/releases/tag/2.1.5>

Pinned TTF archive linked by that release:
<https://github.com/liberationfonts/liberation-fonts/files/7261482/liberation-fonts-ttf-2.1.5.tar.gz>

Upstream source SHA-256 values:

```text
Archive                   7191c669bf38899f73a2094ed00f7b800553364f90e2637010a69c0e268f25d0
LiberationSans-Regular.ttf 76d04c18ea243f426b7de1f3ad208e927008f961dc5945e5aad352d0dfde8ee8
LiberationSans-Bold.ttf    788abee4c806d660e8aee46689dd8540cd4bb98da03dcc9d171ce3efd99a9173
LICENSE                   93fed46019c38bbe566b479d22148e2e8a1e85ada614accb0211c37b2c61c19b
fontconvert.c             42e70906d68e4a7bced1c47a7bb12d2a86c29484eeb564f76449ea7a9b6c3ba1
```

Only maintainers regenerating assets need Python 3, a C compiler, `pkg-config`,
FreeType, and Adafruit GFX **1.12.6**'s `fontconvert` sources. These assets were
rasterized with **FreeType 2.14.3**; changing FreeType can alter pixel hinting.
The generator verifies all source hashes and compiles the pinned converter,
which uses 141 DPI. It also checks glyph bitmap bounds and ASCII coverage.

From the sketch directory, after installing the normal pinned project dependencies:

```sh
python3 scripts/generate_radio_fonts.py --download --check
```

To regenerate rather than check, omit `--check`. To work without downloading,
provide `--archive /path/to/liberation-fonts-ttf-2.1.5.tar.gz` or
`--font-dir /path/to/extracted/font/files` instead of `--download`. Override
`--gfx-dir` if the GFX library is installed outside this project's `.pio` folder.
The archive and TTF sources are temporary build inputs and are not bundled in the
firmware repository.
