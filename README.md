# wled-usermod-controller

[![CI](https://github.com/jmeagher/wled-usermod-controller/actions/workflows/ci.yml/badge.svg)](https://github.com/jmeagher/wled-usermod-controller/actions/workflows/ci.yml)

A [WLED](https://github.com/wled/WLED) usermod that turns a **Waveshare ESP32-C6-Touch-LCD-1.47** (172x320 JD9853 LCD,
AXS5106L capacitive touch) into a stand-alone touch controller for **this WLED and every other WLED on the LAN**:
power, preset and brightness.

> **Status: early.** It compiles and flashes, and the touch controller is detected at boot. The on-screen UI has not
> been verified on hardware yet. Expect rough edges.

## Using it

Main screen (portrait):

| Area | Touch |
|---|---|
| Device row | tap: full-screen device list |
| Power row | tap: toggle |
| Preset row | tap: full-screen preset list |
| Brightness (bottom) | touch or drag anywhere in the area |

The header shows the WiFi network name, the IP address and the number of devices found. For another device, the Power,
Preset and Brightness controls stay blank ("loading..." or "unreachable") until its real state arrives, and ignore touches
until then. The brightness bar fills in proportion to the level and shows the percentage on it.

Lists: drag to scroll, tap an item to choose it, tap **X** (top right) to go back. The screen sleeps after 60 s (the
touch that wakes it does nothing else); an open list closes after 20 s idle. The touch chip is also read while INT is held
low and while the screen is dark, and is reset if it stops answering.

The local device works with no WiFi: power, presets and brightness call WLED directly. Other devices come from WLED's
node list (UDP broadcasts, so *Sync > Show other WLED devices* must be on, which is the default) plus an mDNS scan for
`_wled._tcp`. They are controlled with `POST /json/state`, from a separate task so the UI never blocks. Their presets
are read from `/presets.json`.

## Build and install

Requires `git`, `node`/`npm`, [PlatformIO](https://platformio.org/) (`pio`), and a checkout of
[wled-usermod-ls-formula](https://github.com/jmeagher/wled-usermod-ls-formula) next to this one, whose installer does the
base install (WiFi, name, LED strip). `scripts/install.sh` builds one firmware containing both usermods:

```sh
scripts/install.sh              # build, flash, provision; device "wled-touch" (NAME=, STRIP=, PORT= to override)
scripts/install.sh build        # or just one step: build | flash | provision
CTL_DEBUG=1 scripts/install.sh build   # also log touch events over USB serial
```

To build only this usermod, use `platformio_override.sample.ini` in your own WLED checkout. ESP32-C6 exists only on WLED
`main` (experimental); the pinned commit is in `scripts/dev-setup.sh`.

**Keep the WLED checkout outside this repository.** PlatformIO treats the usermod directory as a library and folds
everything below it in, which hides WLED's `wled_ADCmanager` and the build fails with
`wled_ADCmanager.h: No such file or directory`. The scripts use `~/.cache/wled-usermod-controller/wled`.

## Pins

Defaults (Waveshare docs and demo), editable under *Config > Usermods > Controller* (restart to apply):

| | GPIO |
|---|---|
| LCD SCLK / MOSI / CS / DC / RST / backlight | 1 / 2 / 14 / 15 / 22 / 23 |
| Touch SDA / SCL / RST / INT | 18 / 19 / 20 / 21 |

The LCD is a JD9853: ST7789-compatible but it needs vendor registers, kept in `jd9853_init.h` (generated from Waveshare's
demo). Build with `-D CTL_LCD_PLAIN_ST7789` for a plain ST7789 panel.

## Tests and CI

```sh
tests/run.sh                                   # desktop logic tests (ASan + UBSan, warnings are errors)
python3 -I -m unittest discover -s tests -v    # repository checks
```

The logic that can be tested without a board lives in `ctl_logic.h` (brightness bar, touch decoding, list scrolling);
`tests/test_logic.cpp` checks it, including real touch reports captured from the board, and sanity-checks the JD9853
init table. `tests/test_repo.py` checks things a compile does not: every config key written is read back, the pins in
this README match the code, the pinned WLED commit is the same in the scripts and in CI, `BTNPIN=-1` is kept, scripts
pass `bash -n` and shellcheck.

GitHub Actions (`.github/workflows/ci.yml`) runs those, then builds the firmware against the pinned WLED in three
variants (default, `-D CTL_DEBUG`, `-D CTL_LCD_PLAIN_ST7789`) so flag-only code still compiles, and fails if flash use
passes 90 % of the app partition. A weekly job builds against WLED `main` as an early warning (allowed to fail). CI
cannot test the screen or touch: that still needs a board.

## Not done

Real-hardware verification of the UI; a touch calibration option; the non-touch Waveshare board (button control was removed).
