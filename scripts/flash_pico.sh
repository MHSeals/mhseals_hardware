#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SOURCE_FILE="${REPO_DIR}/pico/thruster_controller.py"
TOOLS_DIR="${REPO_DIR}/.pico-tools"
PORT=""
CHECK_ONLY=false
INSTALL_TOOLS=true

usage() {
    echo "Usage: $0 [--check] [--port DEVICE] [--no-install]"
    echo "Detect a MicroPython Pico and install thruster_controller.py as main.py."
}

while (($#)); do
    case "$1" in
        --check) CHECK_ONLY=true; shift ;;
        --port)
            [[ $# -ge 2 ]] || { echo "--port requires a device" >&2; exit 2; }
            PORT="$2"
            shift 2
            ;;
        --no-install) INSTALL_TOOLS=false; shift ;;
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
    elif ((${#micropython_ports[@]} > 1)); then
        echo "Multiple MicroPython devices found; choose one with --port:" >&2
        printf '  %s\n' "${micropython_ports[@]}" >&2
        exit 1
    else
        mapfile -t acm_ports < <(
            find /dev -maxdepth 1 -type c -name 'ttyACM*' -print 2>/dev/null | sort
        )
        if ((${#acm_ports[@]} == 1)); then
            PORT="${acm_ports[0]}"
        elif ((${#acm_ports[@]} == 0)); then
            echo "No MicroPython or ttyACM device detected." >&2
            exit 1
        else
            echo "Multiple ttyACM devices found; choose the Pico with --port:" >&2
            printf '  %s\n' "${acm_ports[@]}" >&2
            exit 1
        fi
    fi
fi

[[ -e "${PORT}" ]] || { echo "Device does not exist: ${PORT}" >&2; exit 1; }
[[ -r "${PORT}" && -w "${PORT}" ]] || {
    echo "No read/write access to ${PORT}; add the user to the dialout group." >&2
    exit 1
}
echo "Detected Pico candidate: ${PORT} -> $(readlink -f -- "${PORT}")"

if [[ "${CHECK_ONLY}" == true ]]; then
    exit 0
fi

[[ -f "${SOURCE_FILE}" ]] || { echo "Missing ${SOURCE_FILE}" >&2; exit 1; }

if [[ ! -x "${TOOLS_DIR}/bin/python" ]]; then
    if [[ "${INSTALL_TOOLS}" != true ]]; then
        echo "mpremote environment missing; rerun without --no-install." >&2
        exit 1
    fi
    echo "Creating Pico tools environment at ${TOOLS_DIR}"
    python3 -m venv "${TOOLS_DIR}"
    "${TOOLS_DIR}/bin/python" -m pip install --upgrade mpremote
fi

echo "Uploading ${SOURCE_FILE} as main.py"
"${TOOLS_DIR}/bin/python" -m mpremote connect "${PORT}" \
    fs cp "${SOURCE_FILE}" :main.py + reset
echo "Pico flash complete. The controller was reset and is running main.py."
