# Native ODROID hardware configuration

No Pico or MicroPython is used. `mhseals_hardware/config/default.yaml` is
the single shipped configuration. The PWM node, boat test and direct PWM
test load it, then overlay `~/.config/mhseals/hardware.yaml` if present.
Set `MHSEALS_HARDWARE_CONFIG` for a boat-specific file in a mounted workspace
so it survives container recreation. Explicit missing files and unknown keys
fail before hardware opens.

Copy the default YAML into your persistent workspace, edit it, then export:

```sh
export MHSEALS_HARDWARE_CONFIG="$ROS_WS/config/hardware.yaml"
ros2 run mhseals_hardware boat_test
```

Create the directory and copy the file before setting the variable. Keep local
boat overrides out of public Git if they are machine-specific. Never change
pins with outputs armed. Stable platform PWM paths avoid Linux renumbering.
`channel_map` maps FL, FR, RR, RL to physical outputs 1–4; the direct PWM TUI
labels physical outputs, not inferred boat positions.

CLI overrides take precedence for that invocation. Add `--save-config` to
`boat_test` or `thruster_test` to persist pin overrides. For example:

```sh
ros2 run mhseals_hardware boat_test --mosfet-chip gpio3 --mosfet-line 28 --save-config
ros2 run mhseals_hardware boat_test --identify-thrusters
```

The second command **pulses real thrusters**, requires a secured boat and clear,
submerged propellers, and saves the completed channel mapping atomically.
Normal runs reuse the saved/default mapping without repeating identification.
For the ROS node, `--ros-args -p config_file:=/absolute/hardware.yaml` selects
the file; explicit ROS parameters override its values. Changes require restart.

## MOSFET trigger check

The shipped setting is `mosfet_chip: gpio3`, `mosfet_line: 28`,
`mosfet_active_high: true`: **J2 physical pin 11** to the compatible trigger
input, **J2 pin 6** to signal ground for a non-isolated module. See the
[primary-source wiring research](odroid-mosfet-research.md) for electrical
caveats and the possible CAN-overlay conflict. It is high only while hardware
outputs are armed; the passive boat dashboard does not enable it.

Existing saved YAML takes precedence over new defaults. Replace only its
`mosfet_chip: /dev/gpiochip3` with `mosfet_chip: gpio3`; preserve your other
boat settings. Explicit `/dev/gpiochipN` overrides remain supported but bypass
bank-label resolution. All Python entry points use the same shipped YAML.

After pulling, rerun `./scripts/install_odroid_access.sh` **on the host**;
it installs the shared defaults and refreshes permissions for the resolved
bank, including when its device number changed. Custom banks require setting
`MHSEALS_MOSFET_CHIP` in the host service's systemd environment override and
restarting that service to match the runtime YAML. The helper reads only the
plain bank selector from the installed shipped YAML, not your user overrides.
Inside the container rebuild and source the workspace:

```sh
colcon build --packages-select mhseals_hardware
source install/setup.bash
ros2 run mhseals_hardware mosfet_test --mosfet-chip gpio3 --mosfet-line 28
```

Disconnect propulsion power and stop controller processes first. The test
requires typing `ENABLE`, holds inactive for 2 seconds then active for 5 seconds,
and deasserts/releases on normal exit or Ctrl+C. It never writes PWM; any
previously running PWM is not stopped. Measure pin 11 against pin 6 with a
meter, first with the trigger disconnected, then check compatibility with the
module's documented input threshold before connecting it. A GPIO readback is
not a voltage measurement. Use a suitable external pull-down for shutdown or
process failure; software release cannot guarantee the pin remains low.

## Manual keyboard control

`keyboard_control` or the boat-test manual menu: WASD/arrow movement, `+`/`-` speed
in 5% steps (0–100%), Space immediate neutral, X/Escape exit. Default speed is
25%. The node independently neutralizes on missing commands after 0.5 seconds.

The TUI requests Kitty keyboard release events. A compatible terminal and PTY
chain enable multi-key mode: hold W+D for forward/starboard, and add Left/Right
for yaw. Opposite keys cancel; aliases (W and Up) do not double thrust.
Translation diagonals are normalized to the selected translation magnitude;
adding yaw can still increase individual mixer outputs. The speed percentage
is a command scale, not a measured boat speed or a per-motor power limit.
Releasing one key leaves the others active (within the 50 ms input loop).
The TUI explicitly displays whether it has received extended keyboard events.
Use a compatible terminal such as Kitty, including through SSH; intermediate
terminal multiplexers must forward the protocol. Do not assume every terminal
or SSH/client/multiplexer combination supports it.

Ordinary SSH
terminal input has **no key-up signal**: first press gets 0.65 s grace for OS
repeat startup; subsequent repeats expire after 0.15 s. A quick tap in fallback
mode can therefore remain active for the initial grace interval. This mode
deliberately remains single-key: it cannot reliably infer W+D or key release.
Extended mode uses a shared input watchdog so only the newest key needs to
repeat. Input loss clears the chord; repeat events alone cannot re-arm it.
Release and press again after a timeout. OS repeat must be enabled and its
initial delay shorter than the configured initial grace. Excessive network
delay causes a deliberate safety stop, not guaranteed uninterrupted motion.
Focus-out (where reported), Space, exit and input EOF clear all motion. A
terminal that loses a release without reporting focus loss cannot reveal the
true held-key state; retain Space and an independent physical cutoff.

Space is the
explicit immediate stop. Configure both intervals in YAML; do not increase the
repeat timeout to accommodate the first repeat. First test with
`ros2 run mhseals_hardware keyboard_control --dry-run` (no ROS publishing).
Test terminal behavior with
propulsion power disconnected before operating the boat. This is not a physical
emergency stop; loss of the controller/PWM process requires independent hardware
safety provisions.
