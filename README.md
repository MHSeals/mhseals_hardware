# MHS Seals hardware

ROS 2 control for four bidirectional thrusters connected directly to the
ODROID-M2 hardware PWM controllers. There is no intermediary microcontroller
or serial transport.

## Hardware

Physical outputs 1-4 are fixed J2 pins; `channel_map` (canonical FL, FR, RR, RL ->
output) decides which thruster each drives. The boat uses `[1, 3, 2, 4]`.

| Output | J2 pin | Pad | PWM | `pwm_chips` entry (stable platform path) |
| --- | --- | --- | --- | --- |
| 1 | 7  | GPIO0_D0 | PWM7  | `/sys/devices/platform/febd0030.pwm/pwm/pwmchip*` |
| 2 | 12 | GPIO3_B2 | PWM3  | `/sys/devices/platform/fd8b0030.pwm/pwm/pwmchip*` |
| 3 | 15 | GPIO4_B3 | PWM15 | `/sys/devices/platform/febf0030.pwm/pwm/pwmchip*` |
| 4 | 33 | GPIO3_D0 | PWM8  | `/sys/devices/platform/febe0000.pwm/pwm/pwmchip*` |
| MOSFET enable | 13 | GPIO3_D5 | - | `mosfet_chip: gpio3`, `mosfet_line: 29` (active high) |
| Signal ground | 6 | | | |

The table is `odroid_pwm.J2_PINS`, which the TUI also reads.
`pwmchipN` numbers are allocated at boot; the platform glob is stable. `gpio3` is a
bank label resolved to `/dev/gpiochipN` at runtime (line = 8*bank-letter + index,
D5 = 29).
The MOSFET enable moved from pin 11 (GPIO3_D4, line 28) to pin 13 (GPIO3_D5) when the
harness was rewired.

`install_odroid_access.sh` grants access to channel 0 of these four controllers and to
the `mosfet_chip` bank of the *shipped* defaults. Run it again after moving to another
channel or bank.

The MOSFET is the active-high enable. Every hardware entry point configures
neutral PWM before enabling it, and disables the MOSFET before PWM during shutdown.
The GPIO device number is resolved at runtime, not assumed to equal the bank.
`boat_test` uses these same YAML settings: arming enables the MOSFET at neutral;
Space or disarming turns it off.
Connect pin 13 only to a compatible logic-level trigger and pin 6 to its signal
ground; never use the GPIO to power a load or feed a higher voltage back into
the board. Verify the module's input-high threshold with a meter, check for a
CAN or other line consumer, and provide an external inactive-state pull-down
where the module requires one. A GPIO readback is not proof of voltage or
propeller rotation.
See the root [hardware and camera setup](../../README.md#hardware-and-cameras)
section for deployment notes and safety checks.

The device-tree source is
[`config/odroid-m2-thruster-pwm-overlay.dts`](config/odroid-m2-thruster-pwm-overlay.dts).
The running M2 image must expose all four controllers.

## Container setup

The ODROID installation of `astro_dock` mounts `/sys` read-write. Install the
versioned host service once to export the disabled PWM channels at boot and
grant unprivileged access specifically to their control files and bank `gpio3`:

```bash
cd ~/astro_dock/src/mhseals_hardware
./scripts/install_odroid_access.sh
```

The installer is safe to rerun. Recreate the container after changing its
mount configuration. Hardware commands then run as the normal `roboboat`
user; do not use `sudo`.

```bash
cd ~/astro_dock
devcontainer up --workspace-folder .
devcontainer exec --workspace-folder . bash
source install/setup.bash
```

If the workspace has not been built:

```bash
colcon build --packages-select mhseals_hardware
source install/setup.bash
```

## Direct thruster test

Only arm with every thruster submerged, the propeller area and lines clear,
and an emergency stop within reach.

```bash
ros2 run mhseals_hardware thruster_test
```

The program waits for Enter, then holds uninterrupted neutral for three seconds
before enabling live controls. It always neutralizes on exit.

| Key | Action |
| --- | --- |
| Up / Down | Select all or physical outputs 1–4 |
| Left / Right | Decrease/increase pulse width by 10 us |
| `[` / `]` | Decrease/increase pulse width by 1 us |
| `N`, `F`, `B` | Neutral, forward, or reverse preset |
| `Z`, `X` | 1100 or 1900 us endpoint |
| Space or `0` | Neutral immediately |
| `Q` | Neutralize, disable, and exit |

The test frequency is fixed at the ESC-required 50 Hz. Runtime frequency
changes are intentionally prohibited because restarting PWM can be interpreted
as throttle by the ESCs. The terminal reader handles SSH and nested-container
escape sequences and restores terminal state after interruption.

## Thruster identification

Find which physical output drives each thruster, and save it as `channel_map`:

```bash
ros2 run mhseals_hardware thruster_identify
```

With the boat secured and every propeller clear and submerged, type `ARM`.
After five seconds of neutral, each output is pulsed alone at 15% effort. Answer
which thruster moved, which way its propeller turned, and whether its thrust
pointed the expected way; a wrong way is saved as `reversed`. Space, Q or Esc
during a pulse returns every output to neutral. Nothing is saved until you
confirm. `--manual` enters a known mapping without pulses; `--output-backend
topic` identifies the simulator's thrusters. Only `channel_map` and `reversed`
are written. The test suite's Thrusters panel does the same on its page.

## ROS control

Start the hardware node on the ODROID:

```bash
ros2 run mhseals_hardware thruster_pwm_node
```

This arms real outputs. Secure the boat and clear/submerge propellers first.
In another terminal, `ros2 run mhseals_hardware keyboard_control` publishes
manual commands. To select a different **effort** topic, use
`ros2 run mhseals_hardware thruster_pwm_node --ros-args -r cmd_vel:=/control/cmd_vel`.
Run one driver and one selected command publisher only. Inputs are normalized
effort, not measured m/s or rad/s: do not directly connect Nav2's `/nav/cmd_vel`
without a calibrated velocity controller.

It subscribes to `cmd_vel` (`geometry_msgs/msg/Twist`): `linear.x` is forward,
`linear.y` is port/left, and `angular.z` is counterclockwise. A 500 ms command
timeout returns every channel to neutral. Relevant parameters are
`command_timeout`, `channel_map`, `thruster_matrix`, `decoupling`, `reversed`,
`output_backend`, `pwm_topic`, `pwm_chips`, `pwm_channels`, `mosfet_chip`,
`mosfet_line` and `mosfet_active_high`.

The default mixer rows are FL, FR, RR, RL and columns are surge, sway, yaw:

```text
FL  -1  +1  +1
FR  -1  -1  -1
RR  +1  -1  +1
RL  +1  +1  -1
```

The mixer is the geometry and stays editable. `decoupling` (3×3, identity by
default) is applied to the command first and cancels the cross-coupling the
characterization measured. `decoupling_negative` (default unset, same as
`decoupling`) supplies the columns for an axis commanded below zero when the
coupling is one-sided. Results are proportionally desaturated before conversion
to 1100--1900 us. Use `channel_map` to map canonical positions to physical
outputs, for example `[2,4,1,3]`.

`reversed` (FL, FR, RR, RL booleans) flips a thruster that is wired or mounted
backwards; `thruster_identify` and the test suite's thruster set record it.
When started with `config_file`, the node applies edits to the YAML within a
second, without disarming; only the keys the file changed, so a launch override
survives an unrelated edit. A pin, MOSFET or backend edit moves the outputs:
neutral, ESC power off, old channels released, new ones opened at neutral, power
on. If the new outputs fail to open, the old ones reopen and the error is
logged. The node then holds neutral until the next command. For bench tests it
also takes raw pulses per physical output on `~/test_pulses`
(`std_msgs/msg/UInt16MultiArray`, 1100-1900 us), under the same watchdog.

### Simulator outputs

With `output_backend: topic` the node publishes the four pulse widths
(physical outputs 1-4, microseconds, `std_msgs/msg/UInt16MultiArray`) on
`pwm_topic` (default `/thrusters/pwm`) at 20 Hz instead of driving pins. The
mixer, `channel_map` and calibration are applied exactly as on the boat, so
crane_ml (`--crane-ros-thruster-pwm`) responds to the same outputs and
`thruster_identify` works against the simulator too:

```bash
ros2 run mhseals_hardware thruster_pwm_node --ros-args -p output_backend:=topic
```

### Characterization and calibrated velocity control

`thruster_pwm_node` deliberately accepts normalized effort. It does not turn
metres/second into PWM by itself. The fit that does lives in
[`mhseals_testing`](https://github.com/mhseals/mhseals_testing): its
`boat_test` Characterize procedure drives the schedule and saves the result
here on request, and `thrust_characterize` refits any session bag:

```bash
ros2 run mhseals_testing thrust_characterize \
  --bag bags/boat_20261012_101500 \
  --cmd-topic /cmd_vel_effort \
  --config ~/.config/mhseals/hardware.yaml \
  --nav2-params /path/to/mhseals_nav/config/nav2_params.yaml \
  --nav2-overlay ~/.config/mhseals/nav2_calibrated.yaml
```

It fits a speed curve per axis and sign, the body-fixed mechanical bias, acceleration
and per-axis limits into the hardware YAML, reports the day's current (separated
from the bias when the bag covers enough headings), and writes measured Nav2 limits to
pass as `limits_file:=...` to `navigation.launch.py`. The current base profile
remains in `params_file`; partial-effort runs do not extrapolate full-throttle limits. Fit `/cmd_vel_effort` when the bag was
recorded with `--velocity-control`, else the default `/cmd_vel`.
`--apply-decoupling` also writes the fitted `decoupling`, which the driver applies
ahead of the mixer so each axis drives only itself; it hot-reloads, and the
identity matrix turns it off.

The velocity model reads `characterization.speed_curves`, one curve per axis and
sign (`surge+` ... `yaw-`) in wrench units: the effort `e` that holds speed `v`
satisfies `e|e| = d1 v + d2 v|v|`, because T200 thrust goes with `e|e|`. Each
curve must rise up to that axis's velocity limit. A direction without a curve
falls back to the straight line of the older `velocity_gain` diagonal, so a boat
characterized before the curves keeps working until it is characterized again.

After a valid characterization, the optional controller closes the velocity
loop: feed-forward through the model, a world-frame estimate of current and wind
(published on `~/drift`) that stays valid as the boat turns, and PI feedback
with conditional integration. It accepts SI body
velocity on `/cmd_vel`, publishes normalized effort on `/cmd_vel_effort`, and
the PWM node is remapped to that effort topic:

```bash
ros2 run mhseals_hardware calibrated_velocity_controller
ros2 run mhseals_hardware thruster_pwm_node --ros-args -r cmd_vel:=/cmd_vel_effort
```

`ros2 run mhseals_testing boat_test --velocity-control` arms both. The integral
trim is bounded and the watchdog always returns the outputs to neutral.
With `config_file` it rebuilds its model within a second of the YAML changing, and
holds neutral effort while the new model is invalid.

For deadman keyboard control from any machine on the same ROS domain:

```bash
ros2 run mhseals_hardware keyboard_control
```

W/S or Up/Down commands surge, A/D commands sway, Left/Right commands yaw,
Space stops, and X exits. `+`/`-` changes thrust speed in 5% steps. In
`boat_test` manual mode, WASD and the arrows are reserved for motion.
Keys combine when the terminal reports key releases (Kitty protocol, also over
SSH): hold W+D for a diagonal and add ←/→ to rotate; each key stops on its own
release, but a key still held stops about 2 s after another key's release (terminals
do not resume repeating it), so press it again. Otherwise (tmux, plain terminals) the newest key drives alone and stops
when its repeats stop, after an initial-repeat grace. The display shows which mode
is active. Diagonals are normalized, yaw runs at `manual_yaw_scale` (0.5), and the
mixer scales all thrusters together near full effort. Space always stops at once.
Test your terminal safely with
`ros2 run mhseals_hardware keyboard_control --dry-run`.
See the root [hardware and camera setup](../../README.md#hardware-and-cameras)
section for the safety limitations and timeout settings.

## Boat tests

Sensor, actuation, navigation and mission tests live in
[`mhseals_testing`](https://github.com/mhseals/mhseals_testing)
(`ros2 run mhseals_testing boat_test`). It arms this package's
`thruster_pwm_node` and `calibrated_velocity_controller` as supervised
children, and edits the mapping and reversals through the driver's hot reload.

## Development

Shared YAML configuration, pin persistence, and keyboard behavior are documented
in this README. The operator commands are
`keyboard_control` (cmd_vel keyboard publisher), `thruster_pwm_node` (real
outputs), `thruster_test` (direct PWM test), `thruster_identify` (output mapping),
`calibrated_velocity_controller` (m/s to effort), and `command_mux` (MAVROS RC input,
`/cmd_vel/manual` and `/cmd_vel` arbitrated into `/cmd_vel/applied`; the ch6 stop
switch holds, it is not a kill). Run only one cmd_vel controller.

Run the ROS-independent tests with:

```bash
python3 -m pytest -q
```

The package intentionally contains only pin-level and thruster code: the PWM
outputs (pins or a simulator topic), the ROS bridge, the mixer, the velocity
controller, the direct test and identification tools, and their shared
keyboard/manual control code.
