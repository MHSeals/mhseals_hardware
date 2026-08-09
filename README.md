# mhseals_hardware

Minimal ROS 2 to Raspberry Pi Pico thruster control.

## Thruster wiring

Commands and pins always use this order:

| Channel | Position | Pico GPIO |
| --- | --- | --- |
| 1 | Front left (FL) | 11 |
| 2 | Front right (FR) | 12 |
| 3 | Back left (BL) | 13 |
| 4 | Back right (BR) | 14 |

GPIO 15 enables the ESC emergency-stop line. Each ESC uses 50 Hz PWM with
1500 microseconds neutral and a permitted range of 1100--1900 microseconds.

## Pico

Copy `pico/thruster_controller.py` to the Pico as `main.py`. It accepts one
newline-delimited ASCII command at a time over USB serial:

```text
1500,1500,1500,1500
```

Malformed input or 500 ms without a command returns every thruster to neutral.

## ROS 2

Build and run the node from the workspace:

```bash
colcon build --packages-select mhseals_hardware
source install/setup.bash
ros2 run mhseals_hardware thruster_serial_node \
  --ros-args -p serial_port:=/dev/ttyACM0
```

The node subscribes to `cmd_vel` (`geometry_msgs/msg/Twist`) and uses
`linear.x`, `linear.y`, and `angular.z` as surge, sway, and yaw. Mixer results
are proportionally desaturated, converted to ESC pulse widths, and transmitted
in FL, FR, BL, BR order. `baud_rate` (default 115200) and `command_timeout`
(default 0.5 seconds) are ROS parameters.
