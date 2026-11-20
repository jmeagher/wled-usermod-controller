#!/usr/bin/env bash
# Install a controller board: the base install from ../wled-usermod-ls-formula (pinned WLED main, LS Formula
# usermod, WiFi/name/strip provisioning via its board_setup/provision.py) PLUS this controller usermod.
#
#   scripts/install.sh [build|flash|provision|all]      # default: all
#   Board: Waveshare ESP32-C6-Touch-LCD-1.47 (8 MB) -> device "wled-touch". CTL_DEBUG=1 adds touch logging.
#
# Environment: CTL_DEBUG, NAME, PORT, LSF_DIR (the ls-formula checkout), WLED_DIR, STRIP (provision strip spec).
# WiFi credentials come from $LSF_DIR/.wled.secrets (see the ls-formula Readme); never printed here.
#
# Why not just run c6-install.sh? Its build step regenerates platformio_override.ini from the ls-formula
# repo alone, so a second usermod cannot be added. Build+flash happen here with a combined env; the
# installer's `provision` action (which needs no usermod knowledge) is used unchanged.
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
lsf="${LSF_DIR:-$(cd "$here/../wled-usermod-ls-formula" && pwd)}"
dir="${WLED_DIR:-$HOME/.cache/wled-usermod-controller/wled}"
ref="${WLED_REF:-fffa05a765d41d7126b66ab5a3d1ce1f41e7d11e}"   # same pin as the ls-formula installer
action="${1:-all}"
# The controller is touch-only now, so the only supported board is the Waveshare ESP32-C6-Touch-LCD-1.47
# (8 MB flash). Pin defaults live in controller.cpp (LCD SCLK 1 / MOSI 2 / CS 14 / DC 15 / RST 22 / BL 23,
# touch SDA 18 / SCL 19 / RST 20 / INT 21), from Waveshare's docs and demo.
base=esp32c6dev_8MB; name=wled-touch; env_name=touch_esp32c6_8MB
default_strip="type=ws2812,order=grb,pin=7,len=50"   # GPIO 7 is the one header pin Waveshare documents as free
board_flags="-D DATA_PINS=7 -D BTNPIN=-1 ${CTL_DEBUG:+-D CTL_DEBUG}"   # CTL_DEBUG=1: log touch events on the USB serial port
name="${NAME:-$name}"
strip="${STRIP:-$default_strip}"

[ -x "$lsf/board_setup/c6-install.sh" ] || { echo "ls-formula installer not found in $lsf (set LSF_DIR)" >&2; exit 1; }
case "$dir/" in "$here"/*|"$lsf"/*) echo "WLED_DIR ($dir) must be outside both usermod trees" >&2; exit 2 ;; esac

build() {
  "$here/scripts/dev-setup.sh" >/dev/null   # pinned checkout (WLED_REF/WLED_DIR honoured)
  # The override carries both usermods. Each entry must sit on its own line (WLED reads it line by line).
  cat > "$dir/platformio_override.ini" <<INI
[platformio]
default_envs = $env_name

; Touch board.
[env:$env_name]
extends = env:$base
custom_usermods =
  \${env:$base.custom_usermods}
  symlink://$lsf
  symlink://$here
build_flags = \${env:$base.build_flags}
  $board_flags
INI
  (cd "$dir" && { [ -f wled00/html_ui.h ] || { npm ci --silent && npm run build --silent; }; } && pio run -e "$env_name")
}

flash() {
  local port="${PORT:-}"
  if [ -z "$port" ]; then
    local found=()
    for p in /dev/cu.usbmodem* /dev/ttyACM*; do [ -e "$p" ] && found+=("$p"); done
    [ "${#found[@]}" -eq 1 ] || { echo "expected one serial port, found ${#found[@]}; set PORT=" >&2; exit 1; }
    port="${found[0]}"
  fi
  echo "flashing $env_name to $port"
  (cd "$dir" && pio run -e "$env_name" -t upload --upload-port "$port")
}

provision() {
  env ENV_NAME="$env_name" WLED_DIR="$dir" ${PORT:+PORT="$PORT"} \
    "$lsf/board_setup/c6-install.sh" provision --name "$name" --mdns "$name" --strip "$strip"
}

case "$action" in
  build) build ;; flash) flash ;; provision) provision ;;
  all) build; flash; provision ;;
  *) echo "usage: $0 [build|flash|provision|all]" >&2; exit 2 ;;
esac
