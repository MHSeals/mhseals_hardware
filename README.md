# MHS Seals hardware

ROS 2 control for four bidirectional thrusters connected directly to the
ODROID-M2 hardware PWM controllers. There is no intermediary microcontroller
or serial transport.

## Hardware

Thrusters use canonical order `FL, FR, RR, RL`:

```text
pin 7  (FL) ---- pin 12 (FR)
      |             |
pin 33 (RL) ---- pin 15 (RR)
```

The stable PWM controller paths, in canonical order, are:

```text
/sys/devices/platform/febd0030.pwm/pwm/pwmchip*
/sys/devices/platform/fd8b0030.pwm/pwm/pwmchip*
/sys/devices/platform/febf0030.pwm/pwm/pwmchip*
/sys/devices/platform/febe0000.pwm/pwm/pwmchip*
```

Physical header J2 pin 11 (`GPIO3_D4`, bank label `gpio3`, line 28) is the
active-high MOSFET enable. Every hardware entry point configures neutral PWM
before enabling it, and disables the MOSFET before PWM during shutdown.
The GPIO device number is resolved at runtime, not assumed to equal the bank.
See [trigger wiring and voltage checks](docs/odroid-mosfet-research.md) and
[updating existing installations](docs/configuration.md#mosfet-trigger-check).

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
`frequency`, `command_timeout`, `channel_map`, `thruster_matrix`,
`pwm_chips`, `pwm_channels`, `mosfet_chip`, and `mosfet_line`.

The default mixer rows are FL, FR, RR, RL and columns are surge, sway, yaw:

```text
FL  -1  +1  +1
FR  -1  -1  -1
RR  +1  -1  +1
RL  +1  +1  -1
```

Results are proportionally desaturated before conversion to 1100--1900 us.
Use `channel_map` to map canonical positions to physical outputs, for example
`[2,4,1,3]`.

For deadman keyboard control from any machine on the same ROS domain:

```bash
ros2 run mhseals_hardware keyboard_control
```

W/S or Up/Down commands surge, A/D commands sway, Left/Right commands yaw,
Space stops, and X exits. `+`/`-` changes thrust speed in 5% steps.
Release-aware terminals support held combinations: W+D is forward/starboard;
add Left/Right to rotate. Translation diagonals are normalized. Ordinary
terminals explicitly remain single-key, with initial-repeat grace followed by
a short repeat timeout. Test your terminal safely with
`ros2 run mhseals_hardware keyboard_control --dry-run`.
See [configuration](docs/configuration.md)
for the safety limitations and timeout settings. `boat_manual` remains a
compatibility alias; the canonical command matches `keyboard_control.py`.

## Guided boat test

The commissioning dashboard starts **disarmed**, attaching to existing ROS
topics without launching sensors, recording, or opening PWM/GPIO:

```bash
ros2 run mhseals_hardware boat_test
```

Tab cycles overview, sensors, TF, mapping, results, and logs. **H** explicitly
arms/disarms, **M** enters manual control, **1/2/3** characterizes surge/sway/yaw,
**R** edits the persistent map without pulses, **I** identifies outputs with
pulses, **B** toggles recording, and **E** saves a JSON report. Space stops;
Q exits. In manual mode M/X/Esc returns and Space clears motion.

Use `--monitor-only` on a laptop to disable physical/configuration actions.
`--start-stack` explicitly launches real sensor/odometry processes; omit it
when those are already running. `--record` starts recording immediately.
See [the operator guide](docs/boat-test.md) for interlocks, measurements,
topic overrides, mapping/pin configuration, and validation limitations.

Common options:

```bash
ros2 run mhseals_hardware boat_test \
  --start-stack --record \
  --fcu-url serial:///dev/ttyACM0:57600 \
  --require-tf
```

The FCU URL above is MAVROS telemetry and is unrelated to thruster control.
Ctrl+C neutralizes thrusters, stops the hardware node, and flushes the bag.
The tool stops only processes it started. Sensor checks do not replace a
physical power cutoff or independently verify actual motor rotation.

## Development

Shared YAML configuration, pin persistence, and keyboard behavior are documented
in [docs/configuration.md](docs/configuration.md). The operator commands are
`keyboard_control` (cmd_vel keyboard publisher), `thruster_pwm_node` (real
outputs), `thruster_test` (direct PWM test), `boat_test` (guided tests and bags),
and `remote_controller` (MAVROS RC input). Run only one cmd_vel controller.

Run the ROS-independent tests with:

```bash
python3 -m pytest -q
```

The package intentionally contains only the native PWM driver, ROS bridge,
mixer, direct test TUI, guided boat test, and their shared keyboard/manual
control code.
