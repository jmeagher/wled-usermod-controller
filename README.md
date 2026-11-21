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
| Preset row | tap: full-screen preset list (scenes first, presets after) |
| Brightness (bottom) | touch or drag anywhere in the area |

The header shows the WiFi network name, the IP address and the number of devices found. For another device, the Power,
Preset and Brightness controls stay blank ("loading..." or "unreachable") until its real state arrives, and ignore touches
until then. They go blank again when the device stops answering (two failed requests in a row), and when a command
does not arrive the header says so ("<name>: no answer") and the controls wait for the next poll instead of showing
the state that was asked for. The brightness bar fills in proportion to the level and shows the percentage on it.

The selected device stays selected when it drops out of discovery for a while (it is shown as "not found" or
"unreachable" until it answers again); the controller never silently switches to another device. Devices with the same
name get the last part of their address added, e.g. `WLED (.23)`.

Lists: drag to scroll, tap an item to choose it, tap **X** (top right) to go back. The screen sleeps after 60 s (the
touch that wakes it does nothing else); an open list closes after 20 s idle. The touch chip is also read while INT is held
low and while the screen is dark, and is reset if it stops answering.

The preset list puts "scenes" at the top under a SCENES header, followed by the other presets under a PRESETS header;
both groups are sorted by ID. A scene is a WLED playlist, or any preset with a quick-load label (the short "QL" field in
WLED's preset editor, `"ql"` in `presets.json`). Give the label to complete looks you want to pick and leave it off helper
presets that only feed a playlist. Headers appear only when at least one scene exists and are not tappable.
`../wled-usermod-ls-formula`'s device scripts set the label on their scenes.

The local device works with no WiFi: power, presets and brightness call WLED directly. Other devices come from WLED's
node list (UDP broadcasts, so *Sync > Show other WLED devices* must be on, which is the default) plus an mDNS scan for
`_wled._tcp` every 30 s, plus a unicast sweep of the local subnet. Both multicast sources fail on APs or mesh nodes that
do not deliver multicast/broadcast frames to their clients, and some mesh setups do that for only some nodes, so a
device can see an empty node list depending on which AP it joined. The sweep needs only ordinary unicast, so every
device is still found.

The sweep sends a short `GET /json/info` to every address of the subnet (taken from the netmask; a subnet wider than
/22 is clamped to this device's /24) that the node list and mDNS do not already report, about a minute per /24 pass,
then waits a minute. It pauses while the screen is dark and can be turned off with **lan sweep** under
*Config > Usermods > Controller* (on by default; with it off, devices are found only by the node list and mDNS). Each
source lists at most 64 devices.

Other devices are controlled with `POST /json/state` and their presets read from `/presets.json`, from a separate task
so the UI never blocks; every request has a connect timeout, an overall deadline and a size limit, and a tap is sent
before any pending poll or discovery work. Only addresses on this device's subnet are used, whatever mDNS answers.
Note that WLED's JSON API has no authentication: anything on the LAN that answers like a WLED device is listed, and
anyone on the LAN can control the devices this controller can.

## Configuration

Under *Config > Usermods > Controller*:

| Key | Default | |
|---|---|---|
| enabled | on | off: the usermod does nothing (restart to apply) |
| screen timeout s | 60 | seconds of no touch before the screen sleeps; 0 = never |
| lan sweep | on | the unicast sweep of the subnet described above |
| pin ... | see [Pins](#pins) | restart to apply |

If setup fails (a pin already in use, out of memory) the usermod stays off for that boot, says why on this page and,
when the display works, on the screen; `enabled` is left as it was, so fixing the cause and restarting is enough.

## Build and install

Requires `git`, `node`/`npm`, [PlatformIO](https://platformio.org/) (`pio`), and a checkout of
[wled-usermod-ls-formula](https://github.com/jmeagher/wled-usermod-ls-formula) next to this one, whose installer does the
base install (WiFi, name, LED strip). `scripts/install.sh` builds one firmware containing both usermods:

```sh
scripts/install.sh              # build, flash, provision; device "wled-touch" (NAME=, STRIP=, PORT= to override)
scripts/install.sh build        # or just one step: build | flash | provision
CTL_DEBUG=1 scripts/install.sh build   # also log touch events over USB serial (0 or unset: off)
LSF_REF=<commit> scripts/install.sh build   # refuse to build unless the ls-formula checkout is at that commit
```

The ls-formula usermod is linked from its checkout as it is (current branch and uncommitted edits included); the build
prints the commit it used and warns about local changes. The WLED web UI headers are regenerated whenever the WLED
checkout moves to another commit (`WLED_REF`).

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

The display uses the global `SPI` bus and touch the global `Wire` bus. If WLED's own global SPI or I2C pins
(*Config > Usermods*, "Global I2C/SPI GPIOs") are set, they must be the same pins as above (the bus is then shared) or
unset: the ESP32 core starts each bus once and silently ignores a second start on other pins. On a mismatch the
usermod does not start (SPI) or turns touch off (I2C) and says so.

## Tests and CI

```sh
tests/run.sh                                   # desktop tests (ASan + UBSan, warnings are errors)
python3 -I -m unittest discover -s tests -v    # repository checks
```

The logic that can be tested without a board lives in `ctl_logic.h` (brightness bar, touch decoding, list scrolling,
device list merge, subnet and sweep range, request priority, HTTP response head, touch reset timing) and `ctl_json.h`
(preset list filter and parsing). `tests/test_logic.cpp` checks the first, including real touch reports captured from
the board, and sanity-checks the JD9853 init table. `tests/test_json.cpp` checks the second against ArduinoJson 6.18.1,
the version WLED bundles; `tests/run.sh` downloads that single header once into `build/` and verifies its SHA-256 (or
set `ARDUINOJSON_H` to a local copy). `tests/test_repo.py` checks things a compile does not: every config key written is read back, the pins in
this README match the code, the pinned WLED commit is the same in the scripts and in CI, `BTNPIN=-1` is kept, scripts
pass `bash -n` and shellcheck, CI actions are pinned to commit SHAs and its steps run with `pipefail`.

GitHub Actions (`.github/workflows/ci.yml`) runs those, then builds the firmware against the pinned WLED in three
variants (default, `-D CTL_DEBUG`, `-D CTL_LCD_PLAIN_ST7789`) so flag-only code still compiles, and fails if flash use
passes 90 % of the app partition. A weekly job builds against WLED `main` as an early warning (allowed to fail). CI
cannot test the screen or touch: that still needs a board.

## Not done

Real-hardware verification of the UI; a touch calibration option; the non-touch Waveshare board (button control was removed).
