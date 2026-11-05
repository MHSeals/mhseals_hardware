# MHS Seals hardware

ROS 2 control for four bidirectional thrusters connected directly to the
ODROID-M2 hardware PWM controllers. There is no intermediary microcontroller
or serial transport.

| Command | Purpose |
| --- | --- |
| `thruster_pwm_node` | Mixes the mux's `topics.applied` effort into thruster pulses (real pins or a simulator topic) |
| `calibrated_velocity_controller` | Closes the loop from SI velocity on `topics.auto` (`/cmd_vel`) to effort on `topics.effort` (`/cmd_vel_effort`), the mux's auto input |
| `command_mux` | Arbitrates the effort lanes `topics.rc`, `topics.manual` and `topics.effort` into `topics.applied` (rc > manual > hold > auto). The stop switch (`command_mux.stop_ch`) holds the mux; with `estop_cuts_power` true the driver also cuts ESC power on it, latched until the next arm, and with it false the switch only holds |
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
decides which thruster each drives. A vessel's wiring lives in its profile's
`thrusters:` section (blastoise/profile.yaml in the team profile), laid between
`config/default.yaml` and the operator file (see Configuration).

| Output | J2 pin | Pad | PWM |
| --- | --- | --- | --- |
| 1 | 7  | GPIO0_D0 | PWM7  |
| 2 | 12 | GPIO3_B2 | PWM3  |
| 3 | 15 | GPIO4_B3 | PWM15 |
| 4 | 33 | GPIO3_D0 | PWM8  |
| MOSFET enable | 13 | GPIO3_D5 | `mosfet_chip`, `mosfet_line` |
| Signal ground | 6 | | |

The table is `odroid_pwm.J2_PINS`, which the TUI also reads. The addresses live
only in `config/default.yaml`: `pwm_chips` (one stable platform glob per output,
in output order; `pwmchipN` numbers are allocated at boot) and `pwm_channels`.
`mosfet_chip` is a bank label resolved to `/dev/gpiochipN` at runtime.

- Every entry point claims the MOSFET line first (inactive, so a second driver
  fails before it writes any PWM file), configures neutral PWM, then enables
  the MOSFET; on shutdown it disables the MOSFET before PWM.
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

**Bench check of a vessel's profile wiring** (owner present, props off or boat
on stands; agents never run it):

1. Deploy, then start `thruster_pwm_node`: the ready line must name the vessel
   profile, not `vessel thrusters: none`.
2. Only then save a copy of the operator file and remove its `channel_map` (and
   `reversed` if it equals the default).
3. Restart the driver: `~/status` and the ready line show the profile's
   `thrusters: channel_map`.
4. Arm through the normal path (TUI or `boat_agent arm`, props off): the driver
   runs from the session snapshot the arming process froze from its own
   environment, so the snapshot's `channel_map` and the ready line must both show
   the profile's map too.
5. Pulse FL, FR, RR, RL one at a time through the guarded `~/test_pulses` path;
   each must spin the expected motor the expected way.
6. On a wrong thruster restore the saved operator file and fix the profile's
   value, not the operator file.

## Driver

```bash
ros2 run mhseals_hardware thruster_pwm_node
```

This arms real outputs; secure the boat first. It reads `topics.applied`, the
command_mux output (`geometry_msgs/msg/Twist`, normalized **effort**, not m/s):
`linear.x` is forward, `linear.y` port, `angular.z` counterclockwise. After
`command_timeout` (0.5 s) without a command every channel returns to neutral.

The `topics:` section of the YAML names every motion topic once; the mux, the
driver and the controller read it, with no remaps. `topics.auto` (`/cmd_vel`)
is the only SI velocity lane and only the velocity controller reads it; every
mux input is effort. In effort mode an effort publisher drives `topics.effort`.

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
while `topics.mux_state` reports input `idle` (the contract in command_mux).

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
package defaults, the vessel profile's `thrusters:` section (`vessel.py`: the
profile holding `$ASTRO_NETWORK_ENV`; only `channel_map` and `reversed`, any
other key refuses), the operator file, then explicit ROS parameters. A named
vessel that cannot be read (env or a profile.yaml in its chain missing, or not
strict YAML) refuses to load rather than falling back to the defaults. The driver's ready
log line names the `channel_map` and the profile that supplied it, or `none`. Files go through
a strict loader that refuses duplicate keys, anchors, aliases, merge keys and
non-finite numbers, naming `file:line`. Python loading needs no ROS.

`config/schema.yaml` lists the keys the TUI shows with type, units, range, tier
(live or restart) and a one-line doc. Validation is `configuration.validate_config`.

The driver, the mux and the controller live one arm: each reads the arm snapshot
(`config_file`; a `-p` parameter wins over it) once, at startup, and never watches the
YAML, so the driver's e-stop and the mux always share one `command_mux` section. An
edit to the file applies at the next arm. The only live changes go through the
standard SetParameters service, refused with the old value kept when validation fails
and read back with GetParameters: on the driver `decoupling`, `decoupling_negative` (a
`decoupling` set alone carries it along while the two are equal); on `command_mux`
`rc_scale` and `ramp` (applied at once); on the velocity controller its gains. Any other
set, `command_timeout` included (all three stages read it once per arm, so they never
diverge), is refused with "applies at the next arm". The driver's latched
`~/status` reports what is in force under `live`. Agents set them through
`boat_agent set`, so the TUI shows and logs the change.

The `controller:` section holds the velocity controller defaults; the operator
file overrides individual leaves and keeps their siblings:

```yaml
controller:
  kp: [1.2, 0.8, 0.6]
  ki: [0.4, 0.4, 0.2]
  odom_timeout: 0.4
```

Other leaves: `integral_limit`, `feedback`, `drift_time_constant`,
`drift_command_limit`, `drift_limit`, `drift_carry`,
`odom_topic`, `obstacle_topic` (its lanes come from `topics:`). Vectors are
surge, sway, yaw. Unknown leaves and nulls refuse startup. The one top-level
`command_timeout` is the watchdog of all three stages (controller, mux, driver); an old
file's `controller.command_timeout` or `command_mux.cmd_timeout` is dropped on read
with one warning.

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
- `--cmd-topic` is the bag's effort topic, `topics.effort` (`/cmd_vel_effort`);
  effort-mode bags recorded before the `topics:` section carry it on `/cmd_vel`.
- `--apply-decoupling` also writes the fitted `decoupling` (identity turns it off).

The optional controller then applies feed-forward through the model, a
world-frame current and wind estimate (`~/drift`), and bounded PI feedback.
`boat_test --velocity-control` arms it in front of the mux, on the auto lane;
it drives only while the mux input is `auto` and resets otherwise:

```bash
ros2 run mhseals_hardware calibrated_velocity_controller
ros2 run mhseals_hardware command_mux
ros2 run mhseals_hardware thruster_pwm_node
```

## Keyboard control

```bash
ros2 run mhseals_hardware keyboard_control        # --dry-run tests your terminal
```

W/S or Up/Down surge, A/D sway, Left/Right yaw, `+`/`-` change speed in 5%
steps, Space stops, X exits. Yaw runs at `manual_yaw_scale` (0.5); diagonals are
normalized. Terminals that report key releases (Kitty protocol, also over SSH, and through
tmux once its terminal speaks the protocol and `allow-passthrough` is on) combine held keys
and stop each on its release (a key still held stops about 2 s after another key's release,
so press it again). Without releases (xterm, VTE, nested tmux) each key keeps its own
deadline: the newest drives while it repeats, and a key it took over stays `chord_hold`
(0.5 s) after its last event, so recent presses on different axes combine; on one axis the
newest replaces the older. See the root
[hardware and camera setup](../../README.md#hardware-and-cameras) for timeouts.

## Development

```bash
tools/smoke                         # the landing gate (python3 -m pytest -q -m smoke test), under 60 s; --impact runs only affected tests
python3 -m pytest -q                 # the full suite (CI reports it, never blocks)
```

The smoke tier covers config loading and the schema, the mixer, the driver's
parameter guard and the live parameter callbacks; tests that start nodes skip without rclpy.

The package contains only pin-level and thruster code: PWM outputs, the ROS
bridge, mixer, velocity controller, command mux, the test and identification
tools, and their shared keyboard control code.
