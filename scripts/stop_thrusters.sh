#!/usr/bin/env bash
# Interrupt the Pico's running main.py. Its finally block neutralizes PWM.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
TOOLS_PYTHON="${REPO_DIR}/.pico-tools/bin/python"
PORT=""

usage() {
    echo "Usage: $0 [--port DEVICE]"
    echo "Interrupt the Pico thruster controller and leave it stopped at the REPL."
}

while (($#)); do
    case "$1" in
        --port)
            [[ $# -ge 2 ]] || { echo "--port requires a device" >&2; exit 2; }
            PORT="$2"
            shift 2
            ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "${PORT}" ]]; then
    mapfile -t micropython_ports < <(
        find /dev/serial/by-id -maxdepth 1 -type l \
            -iname '*micropython*' -print 2>/dev/null | sort
    )
    if ((${#micropython_ports[@]} == 1)); then
        PORT="${micropython_ports[0]}"
    elif ((${#micropython_ports[@]} == 0)) && [[ -e /dev/ttyACM0 ]]; then
        PORT=/dev/ttyACM0
    else
        echo "Could not identify exactly one Pico; specify --port DEVICE." >&2
        exit 1
    fi
fi

[[ -r "${PORT}" && -w "${PORT}" ]] || {
    echo "No read/write access to ${PORT}." >&2
    exit 1
}
[[ -x "${TOOLS_PYTHON}" ]] || {
    echo "Pico tools are missing; run ./scripts/flash_pico.sh first." >&2
    exit 1
}

"${TOOLS_PYTHON}" - "${PORT}" <<'PY'
import sys
import time

import serial

with serial.Serial(sys.argv[1], 115200, timeout=0.5) as connection:
    connection.write(b'\x03')  # MicroPython KeyboardInterrupt / Ctrl-C
    connection.flush()
    time.sleep(0.2)
    print(connection.read_all().decode(errors='replace'), end='')

print('Pico controller interrupted; PWM outputs were deinitialized.')
PY
