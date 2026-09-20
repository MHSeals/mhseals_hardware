#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly RULE_SOURCE="${SCRIPT_DIR}/../config/99-mhseals-thrusters.rules"
readonly RULE_TARGET="/etc/udev/rules.d/99-mhseals-thrusters.rules"

if [[ ! -f "${RULE_SOURCE}" ]]; then
    echo "Missing udev rule: ${RULE_SOURCE}" >&2
    exit 1
fi
if ! id -nG "${USER}" | tr ' ' '\n' | grep -qx dialout; then
    echo "${USER} must belong to dialout; run: sudo usermod -aG dialout ${USER}" >&2
    exit 1
fi

sudo install -m 0644 "${RULE_SOURCE}" "${RULE_TARGET}"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=gpio --subsystem-match=pwm
sudo udevadm settle

echo "Installed ${RULE_TARGET}"
echo "Recreate the dev container after confirming /sys:/sys:rw is configured."
