# ESP32-S3 internet radio

Internet radio for an ESP32S3-N16R8-DualUSB board, a MAX98357A I²S amplifier, a 1.8-inch ST7735 TFT, and a rotary encoder with a push button.

## Hardware and wiring

The supplied PlatformIO environment targets **16 MB Quad SPI flash and 8 MB Octal SPI PSRAM** (`qio_opi`), with a 240 MHz CPU. This matches an N16R8 board; an N16R8V/WROOM-2 module with Octal SPI flash needs different flash settings.

| Peripheral signal | ESP32-S3 GPIO |
| --- | --- |
| MAX98357A LRC/LRCLK | 4 |
| MAX98357A BCLK | 5 |
| MAX98357A DIN | 6 |
| TFT CS | 10 |
| TFT MOSI/SDA | 11 |
| TFT SCK/SCL | 12 |
| TFT DC/A0 | 13 |
| TFT RESET | 14 |
| Encoder CLK/A | 1 |
| Encoder DT/B | 2 |
| Encoder push button | 42 |

Use a common ground. Encoder inputs use internal pull-ups and the button closes to ground. The TFT configuration is ST7735 black-tab, landscape 160 × 128 pixels. Match power and backlight wiring to the actual breakout modules. GPIO 19/20 remain available for native USB.

The encoder defaults to four quadrature transitions per detent, 25 ms button debounce, and a 700 ms long press. To match a different encoder, append `-DRADIO_INPUT_TICKS_PER_DETENT=2` (or `1`) to the existing `build_flags`. Append `-DRADIO_INPUT_REVERSE=1` if rotation runs in the opposite direction.

## Reproducible build

`platformio.ini` pins the build dependencies:

| Component | Version |
| --- | --- |
| PlatformIO Core | 6.1.18 (installation command below) |
| pioarduino Espressif platform release | 51.03.07 |
| Arduino-ESP32 | 3.0.7 |
| ESP-IDF libraries | 5.1.4, commit `632e0c2a9f` |
| ESP32-audioI2S | tag 3.0.12, commit `928c420d49fce2a09fa91f490b9fcabed6447c67` |
| ArduinoJson | 6.21.6 |
| Adafruit GFX | 1.12.6 |
| Adafruit ST7735/ST7789 | 1.11.0 |
| Adafruit BusIO | 1.17.4 |
| Adafruit seesaw (display-library dependency) | 1.7.9 |
| SD | Arduino-ESP32's bundled 3.0.7 |

The audio library is pinned by Git commit because its release's package manifest reports an older version. Do not replace it with the current library master: newer releases use a different callback API. The platform release's manifest identifies itself as `51.3.6`, but downloads Arduino-ESP32 **3.0.7**; the release URL is intentional.

The explicit SD dependency points to the pinned framework's library. This prevents the display library's generic Arduino SD dependency from shadowing ESP32's incompatible filesystem API.

`default_16MB.csv` uses two application slots of **6,553,600 bytes each** within the 16 MB flash chip. The build checks firmware against the application slot size, rather than treating all physical flash as application space. The preliminary board label still names the generic N8 board; the environment overrides its flash and PSRAM configuration.

`scripts/patch_audioi2s.py` applies a small reviewed fix before compilation: the upstream `connecttohost()` and `httpPrint()` copy the encoded URL allocation length from the shorter unencoded path, reading past its terminating byte. The patch copies only `strlen(path) + 1`. It accepts only the exact pinned `Audio.cpp` SHA256 `ffde34db30d3fbae7a9ac45b8b9fb94d6b89cdd06192a811cd3e57138cd2afbf` or the exact already-patched SHA256 `b41dfb979766f101f39779b009c7d89248ba10cfb6a4bf5d15ac48888d0a5931`. Unexpected source changes stop the build. Reinstalling dependencies reapplies the same patch automatically; the upstream repository and system libraries are not modified.

Run from this directory:

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --index-url https://pypi.org/simple platformio==6.1.18
pio run
```

This downloads the pinned toolchain on the first run. It does not upload firmware. To upload deliberately and inspect logs:

```sh
pio run --target upload --upload-port YOUR_SERIAL_PORT
pio device monitor --port YOUR_SERIAL_PORT --baud 115200
```

Native USB CDC is enabled for `Serial`; use the board's native USB connector for logs. The separate USB-to-UART connector can also upload, but it will not receive `Serial` output with these build flags. Select the appropriate port explicitly when both connectors are attached. If the native USB application port is unavailable, use the board's BOOT/RESET procedure to enter the ROM downloader.

Build outputs are in `.pio/build/esp32s3-n16r8/`: `firmware.bin` is the application image, `firmware.elf` contains debug symbols, and `bootloader.bin`/`partitions.bin` describe the boot and partition setup. `firmware.bin` is not a merged whole-flash image. The PlatformIO upload command supplies the other images and their correct offsets automatically.

## Wi-Fi credentials

Wi-Fi can be configured on the device. An optional compile-time configuration is also supported: copy `secrets.example.h` to `secrets.h` and fill in `WIFI_SSID` and `WIFI_PASSWORD`. `secrets.h` is ignored by Git; the example contains placeholders only.

Without saved or compile-time credentials, the display shows a `GTS-Radio-XXXX` setup network, its randomly generated key, and its local setup address. Join that network from a phone or computer and open **the displayed address using HTTP**. Enter a 2.4 GHz network name and password, then select **Save and connect**. An empty password selects an open network; secured networks accept 8–63 characters or a 64-digit hexadecimal key.

A connection attempt has a 20-second deadline. Failed attempts leave setup available and retry with backoff; the setup AP closes 20 seconds after a successful submitted connection. Open **Menu → Wi-Fi setup** to change networks later. Saved credentials take precedence over the optional compiled defaults.

## Controls and playback

| Screen | Turn | Short press | Hold for 700 ms |
| --- | --- | --- | --- |
| Listening | Change volume; turning also unmutes | Mute/unmute | Open menu |
| Menu | Select an item | Open selected item | Return to listening |
| Stations | Preview station or page/refresh action | Play station or run action | Return to menu |
| Country/Genre/Codec/Max bitrate | Change draft value | Search with draft filter | Cancel and return to menu |
| Wi-Fi setup | — | — | Return to menu |

The menu contains **Stations, Country, Genre, Codec, Max bitrate, Wi-Fi setup, and Retry audio**. A long press fires while held; releasing it does not also produce a short press. Menu/station browsing expires after 15 seconds of inactivity and filter editing after 30 seconds. Wi-Fi setup remains visible without an inactivity timeout.

The station list offers **More stations**, **Previous page**, and **Refresh stations** as applicable. Each directory page contains up to 20 remote rows; unsupported or invalid rows can reduce the visible count. Two curated AIR entries can be added on the first page for Classical/Indian classical with All countries or India, when no explicit codec or bitrate restriction is selected.

Changing a filter searches without retuning the current station. The new filter/list becomes active only after a valid response; network or parse failures keep the previous list. A successful empty response shows no matches. The default search allows all supported codecs without a bitrate cap; menus offer AAC/MP3 and 64–320 kbps caps.

Playback shows connecting, playing, buffering, retrying, failed, or offline states. A working station is restored directly after Wi-Fi connects while directory refresh runs separately. Failed streams receive bounded retries and can fall back to the previous confirmed station; **Retry audio** starts another attempt. Up to eight stations from the last successful first page are cached for startup browsing.

Volume defaults to 6 on a fresh device, with a 0–21 range. Mute preserves the desired listening volume across restarts. A station becomes the saved working station after four seconds continuously in the Playing state; this exceeds the three-second audio freshness window so a single decoded frame cannot replace the previous working station. Settings save after a further 1.5 seconds of inactivity, so allow about six seconds after a station starts before removing power. Old-format settings are migrated when possible; the original firmware never saved pre-mute volume, so that lost value cannot be recovered.

Station names and stream titles scroll within the display. UTF-8 is decoded before fitting text, but the built-in ASCII font has limited glyph coverage: common Latin letters are transliterated and other unsupported characters use `?`.

## Source structure

| File | Responsibility |
| --- | --- |
| `smarthome-radio-esp32s3-max98357a-1.8tft.ino` | Arduino entry points |
| `RadioApp.h` | UI, navigation, worker coordination, deferred settings saves |
| `AudioService.h` | One task owns public Audio calls; callbacks pass copied metadata/frame signals |
| `CatalogService.h` | Worker fetches/validates paginated station lists using verified HTTPS and mirror failover |
| `CatalogState.h` | Hardware-independent catalog request generations, transaction state, timeout, and retry scheduling |
| `SettingsService.h` | NVS worker, checked records, legacy migration, first-page cache |
| `WifiPortal.h` | Wi-Fi attempts/retries and the local setup form |
| `RotaryInput.h` | Polled quadrature/button state machine and bounded input queue |
| `RadioModel.h` | Hardware-independent state, validation, identity, and persistence encoding |
| `RadioConfig.h` | Pin map, timing, available filters |

Directory requests, stream connections, and NVS writes execute outside the UI loop. Public Audio operations stay in the audio-owner task; the library's own decoder/output task is separate. Directory HTTPS waits for a usable SNTP clock and uses the pinned ESP32 core's certificate bundle.

## Library contracts

The pinned [ESP32-audioI2S header](https://github.com/schreibfaul1/ESP32-audioI2S/blob/928c420d49fce2a09fa91f490b9fcabed6447c67/src/Audio.h) supplies the legacy global metadata callbacks. `audio_bitrate` reports bits per second, which the application converts to kbps.

`connecttohost()` and `isRunning()` can report a successful TCP connection before valid audio has been decoded. The application uses the PCM callback to establish that decoded samples are ready. The callback runs before the driver's I²S write, so this does not prove that the amplifier, speaker, or wiring produced sound.

The library starts its own decoder/output task. Callbacks may run from that task, so they must copy metadata or signal flags without drawing, writing preferences, or calling public Audio methods. An `audio_process_i2s` callback must set `*continueI2S = true` to allow audio output.

The pinned audio library disables certificate verification internally for HTTPS audio streams and has no public CA setter. Directory HTTPS verification is configured independently by the application. Provider streams can also use plain HTTP. Updating stream TLS verification requires a compatible library change; this firmware does not claim authenticated audio delivery.

## Validation on the physical radio

A successful build and host tests cannot verify wiring, amplifier output, exact encoder detents, TFT panel offsets, real stream decoder compatibility, or recovery timing. These checks require the assembled radio:

| Check | Expected result |
| --- | --- |
| Boot without credentials; complete the displayed setup form | TFT/input work before connection; setup AP appears; valid credentials connect and survive a restart |
| Set a quiet volume, mute, wait a few seconds, restart, then unmute | Device remains muted after restart and returns to the chosen volume; turning while muted also unmutes |
| Browse stations and edit/cancel a filter while listening | Current audio continues until a station is explicitly selected; cancelled drafts do not change filters |
| Hold the button and rotate quickly | Hold progress updates; one long action occurs; release does not also click; normal turns match physical detents |
| Select More/Previous/Refresh; try a filter with no matches | Pages and cursor remain valid; empty results are distinct from failed searches |
| Make the directory unavailable while a station plays | Playback/controls remain available; old results stay available with an error and retry status |
| Disconnect/reconnect home Wi-Fi | Offline/reconnect state appears; setup remains reachable after failed attempts; the requested station resumes after recovery |
| Select a broken stream, then Retry audio | Connecting/failure states and bounded retries occur; previous confirmed station is available as fallback |
| Read a long or non-ASCII station/title | Text stays inside the screen; scrolling works; unsupported glyphs use a safe replacement |

The USB serial startup log reports detected heap and PSRAM. Confirm the expected 8 MB PSRAM on the actual N16R8 board.

## Host tests

Run every host regression suite with one command:

```sh
python3 tests/run_host_tests.py --sanitize
```

This requires a C++17-capable compiler. Omit `--sanitize` if the compiler does not provide AddressSanitizer and UndefinedBehaviorSanitizer. Set `CXX` to select another compiler. The dependency-patch tests run when the pinned source has been installed by `pio run`; otherwise the runner explicitly reports that those two tests were skipped.

The C++ tests exercise platform-independent state/persistence helpers, encoder/button behavior, catalog request ordering/retry state, Wi-Fi form validation, and the audio service using deterministic Arduino/Audio stubs. They do not require an ESP32 or download Arduino libraries. The audio stub tests verify application state transitions; they do not simulate true concurrent execution or validate the upstream library's internal mutexes.

```sh
c++ -std=c++11 -Wall -Wextra -Werror tests/model_test.cpp -o /tmp/radio-model-test
/tmp/radio-model-test
c++ -std=c++11 -Wall -Wextra -Werror tests/input_test.cpp -o /tmp/radio-input-test
/tmp/radio-input-test
c++ -std=c++11 -Wall -Wextra -Werror tests/catalog_state_test.cpp -o /tmp/radio-catalog-state-test
/tmp/radio-catalog-state-test
c++ -std=c++11 -Wall -Wextra -Werror tests/wifi_portal_test.cpp -o /tmp/radio-wifi-test
/tmp/radio-wifi-test
c++ -std=c++17 -Wall -Wextra -Werror -I tests/audio_stubs tests/audio_service_test.cpp -o /tmp/radio-audio-service-test.bin
/tmp/radio-audio-service-test.bin
python3 tests/audio_patch_test.py
```

The dependency-patch tests require a prior `pio run` to install the pinned source. They verify exact changes, repeat-build idempotence, and refusal to modify an unexpected library revision.
