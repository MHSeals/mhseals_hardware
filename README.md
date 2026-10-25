# MHS Seals hardware

ROS 2 control for four bidirectional thrusters connected directly to the
ODROID-M2 hardware PWM controllers. There is no intermediary microcontroller
or serial transport.

| Command | Purpose |
| --- | --- |
| `thruster_pwm_node` | Mixes `cmd_vel` effort into thruster pulses (real pins or a simulator topic) |
| `calibrated_velocity_controller` | Closes the loop from SI velocity on `/cmd_vel` to effort on `/cmd_vel_effort` |
| `command_mux` | Arbitrates MAVROS RC, `/cmd_vel/manual` and `/cmd_vel` into `/cmd_vel/applied` (rc > manual > hold > auto); the ch6 stop switch holds, it is not a kill |
| `keyboard_control` | Deadman keyboard publisher of effort commands |
| `thruster_test` | Direct per-output PWM test |
| `thruster_identify` | Finds which output drives each thruster and saves `channel_map` / `reversed` |
| `mosfet_test` | Bounded check of the MOSFET enable line |

Run one driver and one selected command publisher only. Sensor, actuation,
navigation and mission tests live in
[`mhseals_testing`](https://github.com/mhseals/mhseals_testing)
(`ros2 run mhseals_testing boat_test`), which arms `thruster_pwm_node` and
`calibrated_velocity_controller` as supervised children.

## Hardware

Outputs 1-4 are fixed J2 pins; `channel_map` (canonical FL, FR, RR, RL -> output)
decides which thruster each drives. The boat uses `[1, 3, 2, 4]`.

| Output | J2 pin | Pad | PWM | `pwm_chips` entry |
| --- | --- | --- | --- | --- |
| 1 | 7  | GPIO0_D0 | PWM7  | `/sys/devices/platform/febd0030.pwm/pwm/pwmchip*` |
| 2 | 12 | GPIO3_B2 | PWM3  | `/sys/devices/platform/fd8b0030.pwm/pwm/pwmchip*` |
| 3 | 15 | GPIO4_B3 | PWM15 | `/sys/devices/platform/febf0030.pwm/pwm/pwmchip*` |
| 4 | 33 | GPIO3_D0 | PWM8  | `/sys/devices/platform/febe0000.pwm/pwm/pwmchip*` |
| MOSFET enable | 13 | GPIO3_D5 | - | `mosfet_chip: gpio3`, `mosfet_line: 29` (active high) |
| Signal ground | 6 | | | |

The table is `odroid_pwm.J2_PINS`, which the TUI also reads. `pwmchipN` numbers
are allocated at boot; the platform glob is stable. `gpio3` is a bank label
resolved to `/dev/gpiochipN` at runtime.

- Every entry point configures neutral PWM before enabling the MOSFET and
  disables the MOSFET before PWM on shutdown.
- Connect pin 13 only to a logic-level trigger and pin 6 to its signal ground.
  Never power a load from the GPIO. Verify the module's input-high threshold
  with a meter and add an external pull-down if the module needs one. A GPIO
  readback is not proof of voltage or propeller rotation.
- The device-tree source is
  [`host/odroid-m2-thruster-pwm-overlay.dts`](host/odroid-m2-thruster-pwm-overlay.dts);
  the running image must expose all four controllers.
- See the root [hardware and camera setup](../../README.md#hardware-and-cameras)
  for deployment notes and safety checks.

## Setup

The ODROID installation of `astro_dock` mounts `/sys` read-write. Install the
host service once; it exports the disabled PWM channels at boot and grants
unprivileged access to their control files and the `mosfet_chip` bank:

```bash
cd ~/astro_dock/src/mhseals_hardware
./scripts/install_odroid_access.sh
```

Rerun it after moving to another channel or bank. Recreate the container after
changing its mounts, then work as the normal `roboboat` user (no `sudo`):

```bash
cd ~/astro_dock
devcontainer up --workspace-folder .
devcontainer exec --workspace-folder . bash
colcon build --packages-select mhseals_hardware   # if not built yet
source install/setup.bash
```

## Direct thruster test

Arm only with every thruster submerged, propellers and lines clear, and an
emergency stop within reach.

```bash
ros2 run mhseals_hardware thruster_test
```

It waits for Enter, holds neutral for three seconds, then enables live
controls, and neutralizes on exit. The frequency is fixed at the ESC-required
50 Hz; restarting PWM can read as throttle to the ESCs.

| Key | Action |
| --- | --- |
| Up / Down | Select all or physical outputs 1-4 |
| Left / Right | Pulse width -/+ 10 us |
| `[` / `]` | Pulse width -/+ 1 us |
| `N`, `F`, `B` | Neutral, forward, reverse preset |
| `Z`, `X` | 1100 or 1900 us endpoint |
| Space or `0` | Neutral immediately |
| `Q` | Neutralize, disable, exit |

## Thruster identification

```bash
ros2 run mhseals_hardware thruster_identify
```

With the boat secured and propellers clear and submerged, type `ARM`. After five
seconds of neutral, each output is pulsed alone at 15% effort. Answer which
thruster moved, which way it turned, and whether thrust pointed the expected
way; a wrong way is saved as `reversed`. Space, Q or Esc returns every output
to neutral, and nothing is saved until you confirm. `--manual` enters a known
mapping without pulses; `--output-backend topic` identifies the simulator's
thrusters. Only `channel_map` and `reversed` are written. The test suite's
Thrusters panel does the same.

## Driver

```bash
ros2 run mhseals_hardware thruster_pwm_node
```

This arms real outputs; secure the boat first. It subscribes to `cmd_vel`
(`geometry_msgs/msg/Twist`, normalized **effort**, not m/s): `linear.x` is
forward, `linear.y` port, `angular.z` counterclockwise. After `command_timeout`
(0.5 s) without a command every channel returns to neutral. To use another
effort topic: `--ros-args -r cmd_vel:=/control/cmd_vel`. Do not connect Nav2's
`/nav/cmd_vel` without the calibrated velocity controller.

Mixer rows are FL, FR, RR, RL; columns are surge, sway, yaw (`thruster_matrix`):

```text
FL  -1  +1  +1
FR  -1  -1  -1
RR  +1  -1  +1
RL  +1  +1  -1
```

- `decoupling` (3x3, identity by default) is applied to the command first and
  cancels the cross-coupling the characterization measured.
  `decoupling_negative` (default unset, same as `decoupling`) gives the columns
  for an axis commanded below zero.
- The result is proportionally desaturated, then converted to 1100-1900 us.
- `reversed` flips a thruster wired or mounted backwards.
- Other parameters: `output_backend`, `pwm_topic`, `pwm_chips`, `pwm_channels`,
  `mosfet_chip`, `mosfet_line`, `mosfet_active_high`.

`~/test_pulses` (`std_msgs/msg/UInt16MultiArray`, 1100-1900 us per physical
output) takes raw bench pulses under the same watchdog. They are accepted only
while `/command_mux/state` (`pulse_guard_topic`; empty disables) reports input
`auto` or `idle`.

**Simulator.** With `output_backend: topic` the node publishes the four pulse
widths (microseconds, `UInt16MultiArray`) on `pwm_topic` (`/thrusters/pwm`) at
20 Hz instead of driving pins. The mixer, `channel_map` and calibration apply
as on the boat, so crane_ml (`--crane-ros-thruster-pwm`) and `thruster_identify`
work against it:

```bash
ros2 run mhseals_hardware thruster_pwm_node --ros-args -p output_backend:=topic
```

## Configuration

Defaults live in `config/default.yaml` (installed to
`share/mhseals_hardware/config/`); the operator file is
`~/.config/mhseals/hardware.yaml` or `$MHSEALS_HARDWARE_CONFIG`. Startup order:
package defaults, operator file, then explicit ROS parameters. Files go through
a strict loader that refuses duplicate keys, anchors, aliases, merge keys and
non-finite numbers, naming `file:line`. Python loading needs no ROS.

`config/schema.yaml` lists the keys the TUI shows with type, units, range, reload
tier and a one-line doc. Validation is `configuration.validate_config`. With
`config_file` set, only `command_timeout` and the `command_mux` keys apply live;
everything else, including pins, backend, `channel_map`, `reversed` and the
mixer, is flagged `restart required` and needs a node restart.

The `controller:` section holds the velocity controller defaults; the operator
file overrides individual leaves and keeps their siblings:

```yaml
controller:
  kp: [1.2, 0.8, 0.6]
  ki: [0.4, 0.4, 0.2]
  odom_timeout: 0.4
```

Other leaves: `integral_limit`, `feedback`, `drift_time_constant`,
`command_timeout`, `drift_command_limit`, `input_topic`, `output_topic`,
`odom_topic`. Vectors are surge, sway, yaw. Unknown leaves and nulls refuse
startup. Controller changes need a restart. Driver `command_timeout`,
`controller.command_timeout` and mux `cmd_timeout` are independent watchdogs.

## Characterization and velocity control

The driver does not turn m/s into PWM. The fit lives in `mhseals_testing`: its
`boat_test` Characterize procedure saves the result to the hardware YAML, and
`thrust_characterize` refits any session bag:

```bash
ros2 run mhseals_testing thrust_characterize \
  --bag bags/boat_20261012_101500 \
  --cmd-topic /cmd_vel_effort \
  --config ~/.config/mhseals/hardware.yaml \
  --nav2-params /path/to/mhseals_nav/config/nav2_params.yaml \
  --nav2-overlay ~/.config/mhseals/nav2_calibrated.yaml
```

- Fits a speed curve per axis and sign (`characterization.speed_curves`), the
  body-fixed bias, acceleration and per-axis limits.
- Reports the day's current, and writes measured Nav2 limits to pass as
  `limits_file:=...` to `navigation.launch.py`.
- Use `--cmd-topic /cmd_vel_effort` when the bag was recorded with
  `--velocity-control`, else `/cmd_vel`.
- `--apply-decoupling` also writes the fitted `decoupling` (identity turns it off).

The optional controller then applies feed-forward through the model, a
world-frame current and wind estimate (`~/drift`), and bounded PI feedback.
`boat_test --velocity-control` arms both:

```bash
ros2 run mhseals_hardware calibrated_velocity_controller
ros2 run mhseals_hardware thruster_pwm_node --ros-args -r cmd_vel:=/cmd_vel_effort
```

## Keyboard control

```bash
ros2 run mhseals_hardware keyboard_control        # --dry-run tests your terminal
```

W/S or Up/Down surge, A/D sway, Left/Right yaw, `+`/`-` change speed in 5%
steps, Space stops, X exits. Yaw runs at `manual_yaw_scale` (0.5); diagonals are
normalized. Terminals that report key releases (Kitty protocol, also over SSH)
combine held keys (a key still held stops about 2 s after another key's release, so press it again); otherwise the newest key drives alone and stops when its
repeats stop. See the root
[hardware and camera setup](../../README.md#hardware-and-cameras) for timeouts.

## Development

```bash
python3 -m pytest -q
```

The package contains only pin-level and thruster code: PWM outputs, the ROS
bridge, mixer, velocity controller, command mux, the test and identification
tools, and their shared keyboard control code.
