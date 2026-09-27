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
ros2 run mhseals_hardware boat_test --mosfet-line 28 --save-config
ros2 run mhseals_hardware boat_test --identify-thrusters
```

The second command **pulses real thrusters**, requires a secured boat and clear,
submerged propellers, and saves the completed channel mapping atomically.
Normal runs reuse the saved/default mapping without repeating identification.
For the ROS node, `--ros-args -p config_file:=/absolute/hardware.yaml` selects
the file; explicit ROS parameters override its values. Changes require restart.

## Manual keyboard control

`keyboard_control` or the boat-test manual menu: WASD/arrow movement, `+`/`-` speed
in 5% steps (0–100%), Space immediate neutral, X/Escape exit. Default speed is
25%. The node independently neutralizes on missing commands after 0.5 seconds.

The TUI requests Kitty keyboard release events. A compatible terminal and PTY
chain allow neutral on release (within the 50 ms input loop). Ordinary SSH
terminal input has **no key-up signal**: first press gets 0.65 s grace for OS
repeat startup; subsequent repeats expire after 0.15 s. A quick tap in fallback
mode can therefore remain active for the initial grace interval. Space is the
explicit immediate stop. Configure both intervals in YAML; do not increase the
repeat timeout to accommodate the first repeat. Test terminal behavior with
propulsion power disconnected before operating the boat. This is not a physical
emergency stop; loss of the controller/PWM process requires independent hardware
safety provisions.
