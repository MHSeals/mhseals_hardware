#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly CONFIG_DIR="${SCRIPT_DIR}/../config"
readonly HELPER_SOURCE="${CONFIG_DIR}/mhseals-thruster-access"
readonly HELPER_TARGET="/usr/local/libexec/mhseals-thruster-access"
readonly UNIT_SOURCE="${CONFIG_DIR}/mhseals-thruster-access.service"
readonly UNIT_TARGET="/etc/systemd/system/mhseals-thruster-access.service"
readonly LEGACY_RULE="/etc/udev/rules.d/99-mhseals-thrusters.rules"

for source in "${HELPER_SOURCE}" "${UNIT_SOURCE}"; do
    [[ -f "${source}" ]] || { echo "Missing access file: ${source}" >&2; exit 1; }
done

sudo install -D -m 0755 "${HELPER_SOURCE}" "${HELPER_TARGET}"
sudo install -m 0644 "${UNIT_SOURCE}" "${UNIT_TARGET}"
sudo rm -f "${LEGACY_RULE}"
sudo systemctl daemon-reload
sudo systemctl enable --now mhseals-thruster-access.service

echo "Installed and started mhseals-thruster-access.service"
echo "Recreate the dev container after confirming /sys:/sys:rw is configured."
