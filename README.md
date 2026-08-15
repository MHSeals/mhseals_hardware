# mhseals_hardware

Minimal ROS 2 to Raspberry Pi Pico thruster control.

## Thruster wiring

Commands and pins always use this order:

| Channel | Position | Pico GPIO |
| --- | --- | --- |
| 1 | Front left (FL) | 11 |
| 2 | Front right (FR) | 12 |
| 3 | Rear right (RR) | 13 |
| 4 | Rear left (RL) | 14 |

GPIO 15 enables the ESC emergency-stop line. Each ESC uses 50 Hz PWM with
1500 microseconds neutral and a permitted range of 1100--1900 microseconds.

## Pico

Copy `pico/thruster_controller.py` to the Pico as `main.py`. It accepts one
newline-delimited ASCII command at a time over USB serial:

```text
1500,1500,1500,1500
```

Malformed input or 500 ms without a command returns every thruster to neutral.

From the Odroid, the repository helper detects the MicroPython USB device,
creates a local `mpremote` environment on first use, uploads the controller as
`main.py`, and resets the Pico:

```bash
cd ~/astro_dock/src/mhseals_hardware
./scripts/flash_pico.sh --check
./scripts/flash_pico.sh
```

Detection prefers the stable `/dev/serial/by-id/*MicroPython*` link. When more
than one serial device is connected, select it explicitly with `--port`.
Keep the boat secured and thrusters clear and submerged before flashing:
resetting the Pico starts `main.py` and enables the ESC control line.

## ROS 2

Build and run the node from the workspace:

```bash
colcon build --packages-select mhseals_hardware
source install/setup.bash
ros2 run mhseals_hardware thruster_serial_node \
  --ros-args -p serial_port:=/dev/ttyACM0
```

The node subscribes to `cmd_vel` (`geometry_msgs/msg/Twist`) and uses the ROS
body convention: `linear.x` is forward surge, `linear.y` is port/left sway,
and `angular.z` is counterclockwise yaw viewed from above. Mixer results are
proportionally desaturated and converted to ESC pulse widths.

The normalized starting matrix has rows in FL, FR, RR, RL order and columns in
surge, sway, yaw order:

```text
FL  -1  +1  +1
FR  -1  -1  -1
RR  +1  -1  +1
RL  +1  +1  -1
```

This follows the 45-degree geometry and the installed polarity: positive local
thrust on the front pair contributes aft force, while positive local thrust on
the rear pair contributes forward force. Override the flattened row-major
matrix with `thruster_matrix`. Use `channel_map` to map canonical FL,FR,RR,RL
positions onto physical Pico outputs; for example `[2,4,1,3]` sends FL to
output 2, FR to 4, RR to 1, and RL to 3.

## Bagged boat test

Only run the test with every thruster submerged, propellers and lines clear,
an accessible emergency stop, and a second person watching the water. Build
both packages and start the guided test:

```bash
colcon build --packages-select mhseals_hardware mhseals_nav
source install/setup.bash
ros2 run mhseals_hardware boat_test
```

The runner asks for the Pico device and MAVROS FCU URL, starts a minimal
MAVROS measurement launch, and records every topic to a timestamped directory
under `bags/`. Its live terminal dashboard checks actual message arrival for
odometry, GPS, and IMU, and keeps status indicators for optional LiDAR and
camera data. Optional drivers are not started unless `--optional-sensors` is
given, so a missing ZED or Velodyne installation cannot prevent the first
boat test. The runner also refuses to compete with another `cmd_vel`
publisher.

The displayed numbering is fixed in canonical software order:

```text
1 (FL) ---- 2 (FR)
   |          |
4 (RL) ---- 3 (RR)
```

If no `--channel-map` is supplied, the setup pulses each physical output at
15 percent, asks which position moved, verifies polarity, and applies the
result in software. Save the printed map for later runs:

```bash
ros2 run mhseals_hardware boat_test \
  --serial-port /dev/ttyACM0 \
  --fcu-url serial:///dev/ttyACM1:57600 \
  --channel-map 2,4,1,3
```

The characterization menu runs three positive and three negative 25-percent,
three-second trials per selected axis. It uses a balanced `+,-,-,+,+,-` order
with five seconds of neutral data before and after each command. Before every
trial it states the required clearance and expected movement:

| Test | Positive direction | Negative direction | Required clearance |
| --- | --- | --- | --- |
| Surge | Forward | Reverse | Ahead or astern respectively |
| Sway | Port/left | Starboard/right | On the commanded side |
| Yaw | Counterclockwise | Clockwise | Full boat perimeter; loose lines |

Test phases, completion markers, matrix values, and channel mapping are
recorded as JSON on `/boat_test/events`. Ctrl+C commands neutral, stops hardware control,
flushes the bag, and then stops the sensor stack. Confirm the result with:

```bash
ros2 bag info bags/boat_test_YYYYMMDD_HHMMSS
```

## Hand tuning for reduced drift

For each axis, use the five-second neutral windows to estimate environmental
drift. Compare the three positive and three negative trials after subtracting
that baseline, focusing on displacement and yaw outside the commanded axis.
Adjust only that axis's column in `thruster_matrix`, normalize its largest
absolute coefficient to 1.0, and repeat the same profile. Supply a candidate
matrix as 12 comma-separated row-major values:

```bash
ros2 run mhseals_hardware boat_test \
  --thruster-matrix=-1,1,1,-1,-1,-1,1,-1,1,1,1,-1
```

A single linear matrix may need to be a compromise because forward and reverse
thrust differ. Direction-dependent compensation and automatic fitting are
intentionally deferred until the collected boat data has been reviewed.
