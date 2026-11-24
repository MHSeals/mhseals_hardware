# mhseals_hardware

ROS 2 control for four bidirectional thrusters wired straight to the ODROID-M2's hardware PWM. In the
simulator, the same pulses go out on a topic instead.

```bash
ros2 launch mhseals_hardware drive.launch.py                          # boat: ODROID pins
ros2 launch mhseals_hardware drive.launch.py output_backend:=topic use_sim_time:=true   # sim
ros2 run mhseals_hardware mix 0.5 0 0                                 # what surge 0.5 becomes
```

## The stack

`drive.launch.py` is the only way the stack starts. It runs:

| Node | Does |
| --- | --- |
| `thruster_pwm_node` | Mixes `topics.applied` into four pulses (docs/CONTROL.md) on the pins or `pwm_topic`. Always publishes what it writes on `pwm_topic` (`/thrusters/pwm`, UInt16MultiArray, outputs 1-4, µs, 20 Hz). After `command_timeout` (0.5 s) with no command, it sets every output to neutral |
| `command_mux` | Picks one lane, duplicate > rc > manual > stop > auto > idle, and publishes it on `topics.applied`. RC MANUAL or a stick takeover wins at once. A lane that goes stale falls through to the next, so nothing stays locked |
| `calibrated_velocity_controller` | Started only with `controller:=true`. Turns Nav2 body velocity (`topics.auto`) into effort (`topics.effort`). It refuses in one line when there is no characterization yet |

Launch arguments: `output_backend` (`odroid` or `topic`; empty keeps the config value), `use_sim_time`,
`controller`, `config_file`.

Effort is normalised: Twist `linear.x` is forward, `linear.y` is port and `angular.z` is
counterclockwise, each in [-1, 1]. Anything published on `topics.effort` (`/cmd_vel_effort`) drives
through the mux's auto lane. `astro test` uses that lane.

Losing an input sends neutral, and nothing stays locked afterwards: whoever next means to move the
boat moves it at once. The stops that do hold are the ones someone chooses, and each one clears in a
known way:

- **RC mode switch at HOLD:** holds while it is there. Moving the switch out clears it.
- **RC stop channel:** cuts ESC power and latches (`estop_cuts_power`). Restart `drive.launch` to
  clear it.
- **`astro estop`** (`ros2 service call /thruster_pwm_node/estop std_srvs/srv/Trigger`): the same
  latched cut. Restart `drive.launch` to clear it.

Losing RC never cuts power. The command timeout (0.5 s) brings neutral.

docs/authority.md has the details.

## Commands

| `astro` verb | Command | Does |
| --- | --- | --- |
| `drive` | `ros2 launch mhseals_hardware drive.launch.py` | Starts the stack |
| `estop` | `ros2 service call /thruster_pwm_node/estop std_srvs/srv/Trigger` | Stops the thrusters now (ESC power cut). Restart `drive.launch` to clear it |
| `mix` | `ros2 run mhseals_hardware mix S W Y` / `--live` | Prints every mixer stage for a command, or follows `topics.applied` |
| `config_check` | `ros2 run mhseals_hardware config_check FILE` | Validates a hardware YAML through the driver's loader. Exits 0, or prints the reason and exits 1 |
| `keyboard_control` | `ros2 run mhseals_hardware keyboard_control` | Deadman keys on the manual lane: WASD or arrows, Space neutral, X exit |
| `wiring` | `ros2 run mhseals_hardware thruster_identify` | Pulses each output and asks what moved, then saves `channel_map` and `reversed`. Stop `drive` first |
| `thruster_test` | `ros2 run mhseals_hardware thruster_test` | Sets each output's pulse by hand. Stop `drive` first |
| `mosfet_test` | `ros2 run mhseals_hardware mosfet_test` | Holds the MOSFET enable on for a bounded time. Writes no PWM |

`help.yaml` lists the arguments (`astro help mhseals_hardware`). Secure the boat and clear the
propellers before anything that drives thrusters.

## Configuration

Each layer is loaded over the one before it:

1. `config/default.yaml`: structure and defaults, kept in git. `config/schema.yaml` documents
   each key.
2. The vessel profile's `thrusters:` section: only `channel_map` and `reversed`.
3. `~/.config/mhseals/hardware.yaml` (or `$MHSEALS_HARDWARE_CONFIG`, or `config_file:=`): the measured
   and tuned state. `astro apply` writes it.

The loader is strict. It refuses unknown keys, duplicate keys, anchors and non-finite numbers, and
names `file:line`. Nodes read the config once, when they start. Only a few values change live through
`ros2 param set`:

- on `thruster_pwm_node`: `decoupling` and `decoupling_negative`;
- on `calibrated_velocity_controller`: its gains;
- on `command_mux`: `rc_scale` and `ramp`.

docs/CONTROL.md maps each key to its layer and to the test that sets it.

## Hardware

Outputs 1-4 are fixed J2 pins. `channel_map` decides which thruster each one drives.

| Output | J2 pin | PWM |
| --- | --- | --- |
| 1 | 7  | PWM7  |
| 2 | 12 | PWM3  |
| 3 | 15 | PWM15 |
| 4 | 33 | PWM8  |
| MOSFET enable | 13 | `mosfet_chip`, `mosfet_line` |
| Signal ground | 6 | |

The board addresses (`pwm_chips`, `pwm_channels`, `mosfet_*`) are set only in `config/default.yaml`.
To change one, edit it there and rerun `scripts/install_odroid_access.sh`. Run that script once per
ODROID, on the host. It exports the PWM channels at boot and grants the container access to them.
Every entry point claims the MOSFET line first, sets neutral, and only then enables it. On exit it
turns the MOSFET off before PWM.

## Tests

```bash
python3 -m pytest -q        # pure Python, no ROS, a few seconds
```

The tests cover the core building blocks: the mixer, the velocity law, mux arbitration, the
e-stop, manual state, configuration loading, and the ODROID PWM backend.
