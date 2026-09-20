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

Physical header pin 11 (`GPIO3_D4`, `/dev/gpiochip3` line 28) is the
active-high MOSFET enable. Every hardware entry point configures neutral PWM
before enabling it, and disables the MOSFET before PWM during shutdown.

The device-tree source is
[`config/odroid-m2-thruster-pwm-overlay.dts`](config/odroid-m2-thruster-pwm-overlay.dts).
The running M2 image must expose all four controllers.

## Container setup

The ODROID installation of `astro_dock` mounts `/sys` read-write. Install the
versioned host service once to export the disabled PWM channels at boot and
grant unprivileged access specifically to their control files and `gpiochip3`:

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

The program waits for Enter before arming and always neutralizes on exit.

| Key | Action |
| --- | --- |
| Up / Down | Select all, FL, FR, RR, or RL |
| Left / Right | Decrease/increase pulse width by 10 us |
| `[` / `]` | Decrease/increase pulse width by 1 us |
| `N`, `F`, `B` | Neutral, forward, or reverse preset |
| `Z`, `X` | 1100 or 1900 us endpoint |
| `-`, `+` | Change frequency; outputs reset to neutral |
| Space or `0` | Neutral immediately |
| `Q` | Neutralize, disable, and exit |

The terminal reader handles SSH and nested-container escape sequences and
restores terminal state after interruption.

## ROS control

Start the hardware node on the ODROID:

```bash
ros2 run mhseals_hardware thruster_pwm_node
```

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
ros2 run mhseals_hardware boat_manual
```

W/S or Up/Down commands surge, A/D commands sway, Left/Right commands yaw,
Space stops, and X exits. Commands expire after 350 ms unless keys continue
arriving.

## Guided boat test

The guided workflow checks sensor traffic, identifies physical thruster
positions, runs repeatable surge/sway/yaw trials, and records ROS bags:

```bash
ros2 run mhseals_hardware boat_test
```

Use `--allow-missing-sensors` for a secured thruster-only bring-up. If no
`--channel-map` is given, the workflow pulses each output at 15 percent and
asks which position moved. Its test menu supports surge, sway, yaw, all tests,
and the same deadman manual controller. Process logs and bags are stored under
`bags/`.

Common options:

```bash
ros2 run mhseals_hardware boat_test \
  --fcu-url serial:///dev/ttyACM0:57600 \
  --channel-map 2,4,1,3 \
  --allow-missing-sensors
```

The FCU URL above is MAVROS telemetry and is unrelated to thruster control.
Ctrl+C neutralizes thrusters, stops the hardware node, and flushes the bag.

## Development

Run the ROS-independent tests with:

```bash
python3 -m pytest -q
```

The package intentionally contains only the native PWM driver, ROS bridge,
mixer, direct test TUI, guided boat test, and their shared keyboard/manual
control code.
