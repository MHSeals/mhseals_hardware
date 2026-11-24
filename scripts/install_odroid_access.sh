#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly HOST_DIR="${SCRIPT_DIR}/../host"
readonly HELPER_SOURCE="${HOST_DIR}/mhseals-thruster-access"
readonly HELPER_TARGET="/usr/local/libexec/mhseals-thruster-access"
readonly UNIT_SOURCE="${HOST_DIR}/mhseals-thruster-access.service"
readonly UNIT_TARGET="/etc/systemd/system/mhseals-thruster-access.service"
readonly DEFAULT_SOURCE="${SCRIPT_DIR}/../config/default.yaml"
readonly LEGACY_RULE="/etc/udev/rules.d/99-mhseals-thrusters.rules"

for source in "${HELPER_SOURCE}" "${UNIT_SOURCE}" "${DEFAULT_SOURCE}"; do
    [[ -f "${source}" ]] || { echo "Missing access file: ${source}" >&2; exit 1; }
done

# The pins the boot service prepares: default.yaml < the vessel profile's thrusters: (ROS_WS, default
# the workspace holding this package). Without python3-yaml or a vessel, default.yaml's.
pins=$(mktemp)
trap 'rm -f "${pins}"' EXIT
if ROS_WS=${ROS_WS:-$(cd -- "${SCRIPT_DIR}/../../.." && pwd)} PYTHONPATH="${SCRIPT_DIR}/.." python3 -c '
from mhseals_hardware import vessel
from mhseals_hardware.configuration import load_defaults, validate_config
c = validate_config(dict(load_defaults(), **vessel.section("thrusters")))
print("pwm_chips:")
print("".join(f"  - {g}\n" for g in c["pwm_chips"]), end="")
print("pwm_channels: [%s]" % ", ".join(map(str, c["pwm_channels"])))
print("mosfet_chip:", c["mosfet_chip"])
' >"${pins}" 2>/dev/null; then
    echo "Pins from the vessel profile and default.yaml:"; cat "${pins}"
else
    echo "Could not read the vessel profile (python3-yaml?); installing default.yaml's pins" >&2
    cp "${DEFAULT_SOURCE}" "${pins}"
fi

sudo install -D -m 0755 "${HELPER_SOURCE}" "${HELPER_TARGET}"
sudo install -D -m 0644 "${pins}" /usr/local/share/mhseals/hardware-default.yaml
sudo install -m 0644 "${UNIT_SOURCE}" "${UNIT_TARGET}"
sudo rm -f "${LEGACY_RULE}"
sudo systemctl daemon-reload
sudo systemctl enable --now mhseals-thruster-access.service
sudo systemctl restart mhseals-thruster-access.service

echo "Installed and started mhseals-thruster-access.service"
echo "Recreate the dev container after confirming /sys:/sys:rw is configured."
