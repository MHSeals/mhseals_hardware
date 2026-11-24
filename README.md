# mhseals_hardware

Drives the boat's four thrusters from ROS 2: a command mux picks who drives, a mixer turns the command into
four pulses on the ODROID-M2's hardware PWM. In the simulator the same pulses go out on a topic.

```bash
astro up drive                      # driver + mux on the boat computer (drive.launch.py)
astro up drive --with controller    # ... plus the velocity controller (needs a characterization)
astro mix 0.5 0 0                   # what surge 0.5 becomes, stage by stage
astro estop                         # cut ESC power now; astro estop clear powers it again
astro thrusters                     # drive each output, set wiring and pins (the thrusters screen)
```

How a command becomes thrust, layer by layer, and which test sets each layer:
[docs/CONTROL.md](docs/CONTROL.md). Who may drive and what stops the boat:
[docs/authority.md](docs/authority.md).

## The stack

`drive.launch.py` is the only way the stack starts. It runs:

| Node | Does |
| --- | --- |
| `thruster_pwm_node` | Mixes `topics.applied` into four pulses (docs/CONTROL.md) on the pins or `pwm_topic`. Always publishes what it writes on `pwm_topic` (`/thrusters/pwm`, UInt16MultiArray, outputs 1-4, µs, 20 Hz). After `command_timeout` (0.5 s) with no command, it sets every output to neutral |
| `command_mux` | Picks one lane, duplicate > rc > stop > manual > auto > idle, and publishes it on `topics.applied`. RC MANUAL or a stick takeover wins at once. A lane that goes stale falls through to the next, so nothing stays locked |
| `calibrated_velocity_controller` | Started only with `controller:=true`. Turns Nav2 body velocity (`topics.auto`) into effort (`topics.effort`). It refuses in one line when there is no characterization yet |

Launch arguments: `output_backend` (`odroid` or `topic`; empty keeps the config value), `use_sim_time`,
`controller`, `config_file`. The sim starts it with `output_backend:=topic use_sim_time:=true`.

Effort is normalised: Twist `linear.x` is forward, `linear.y` is port and `angular.z` is
counterclockwise, each in [-1, 1]. Anything published on `topics.effort` (`/cmd_vel_effort`) drives
through the mux's auto lane. `astro test` uses that lane.

Losing an input sends neutral, and nothing stays locked: whoever next means to move the boat moves it
at once. Only chosen stops hold, and each has one command that removes it:

| Blocks the outputs | Removes it |
| --- | --- |
| RC mode switch at HOLD | move the switch out of HOLD |
| RC stop channel: ESC power cut and latched (`estop_cuts_power`, on by default) | flip the switch up, then `astro estop clear` |
| `astro estop`: the same latched cut | `astro estop clear` |
| heartbeat stall latch (only with `mosfet_heartbeat` > 0) | `astro estop clear` |
| another program holds the pins or the MOSFET line | `astro thrusters free` |
| command timeout (neutral) | send any command |

`astro estop clear` powers the ESCs again at neutral; it refuses while the stop switch is still down.
Losing RC never cuts power; the command timeout (0.5 s) brings neutral.

## Commands

| `astro …` | Plain ROS 2 | Does |
| --- | --- | --- |
| `drive` | `ros2 launch mhseals_hardware drive.launch.py` | Starts the stack |
| `estop` | `ros2 service call /thruster_pwm_node/estop std_srvs/srv/Trigger` | Stops the thrusters now (ESC power cut, latched) |
| `estop clear` | `ros2 service call /thruster_pwm_node/clear std_srvs/srv/Trigger` | Powers the ESCs again at neutral; refused while the RC stop switch is down |
| `mix` | `ros2 run mhseals_hardware mix S W Y` / `--live` | Prints every mixer stage for a command, or follows `topics.applied` |
| `keyboard` | `ros2 run mhseals_hardware keyboard` | Deadman keys on the manual lane: WASD or arrows, Space neutral, X exit |
| `thrusters` (`test`, `config`) | `ros2 run mhseals_hardware thrusters` | The thrusters screen (below) |
| `thrusters wiring` | `ros2 run mhseals_hardware thrusters wiring` | Pulses each output and asks what moved, then saves `channel_map` and `reversed` to hardware.yaml |
| `thrusters mosfet` | `ros2 run mhseals_hardware thrusters mosfet [--seconds N]` | Holds the MOSFET enable (ESC power) on until Ctrl-C, SIGTERM or `--seconds`. Writes no PWM |
| `thrusters free` | `ros2 run mhseals_hardware thrusters free [--check]` | Stops every program holding the pins or MOSFET, sets neutral, releases them, and prints one row per output with its holder. Exit 0 when all are free |
| `config_check` | `ros2 run mhseals_hardware config_check FILE` | Setup: validates a hardware YAML through the driver's loader. Exits 0, or prints the reason and exits 1 |
| `odroid_access` | `scripts/install_odroid_access.sh` | Setup: once per ODROID, on the host (below) |

The nodes `thruster_pwm_node`, `command_mux` and `calibrated_velocity_controller` start only through
`drive`. Every entry that touches the pins or ROS runs on the boat computer: astro forwards it there.
`help.yaml` lists the arguments (`astro help mhseals_hardware`). Secure the boat and clear the
propellers before anything that drives thrusters.

### The thrusters screen

`astro thrusters` shows one row per thruster (FL, FR, RR, RL): its output, J2 pin, PWM chip (✓ when it
resolves on this host), reversed, the commanded pulse and effort with a strength bar centred on neutral,
and the pulse read back from sysfs. The header shows the MOSFET and who holds it, whether `drive` is
running, and the ESC arming countdown.

The first pulse opens the outputs: the MOSFET goes on and every ESC is held at neutral for 3 s while it
arms. Keys (also on the bottom line): ↑/↓ select a thruster or ALL, ←/→ ±10 µs, [/] ±1 µs, f/b ±25 %
(reversal honoured), 0 or Space neutral, z/x 1100/1900 µs, t the output this thruster is on, r reversed,
p the PWM chip (from the chips this host exposes), m the MOSFET, k free (stop what holds the pins),
s save, q quit at neutral with the MOSFET off. s shows the diff of the vessel profile's `thrusters:` and
y writes it; commit it in the profiles repo.

## Configuration

Each layer is loaded over the one before it:

1. `config/default.yaml`: structure and defaults, kept in git. `config/schema.yaml` documents
   each key.
2. The vessel profile's `thrusters:` section: `channel_map`, `reversed` and the pins (`pwm_chips`,
   `pwm_channels`, `mosfet_chip`, `mosfet_line`, `mosfet_active_high`).
3. `~/.config/mhseals/hardware.yaml` (or `$MHSEALS_HARDWARE_CONFIG`, or `config_file:=`): the measured
   and tuned state. `astro apply` writes it.

The loader is strict. It refuses unknown keys, duplicate keys, anchors and non-finite numbers, and
names `file:line`. Nodes read the config once, when they start. Only a few values change live through
`ros2 param set`:

- on `thruster_pwm_node`: `decoupling` and `decoupling_negative`;
- on `calibrated_velocity_controller`: its gains;
- on `command_mux`: `rc_scale` and `ramp`.

[docs/CONTROL.md](docs/CONTROL.md) maps each key to its layer and to the test that sets it.
`config/tests.yaml` holds those tests (what `astro test list` shows).

## Hardware

Outputs 1-4 are J2 pins (the defaults below; a vessel profile may move them). `channel_map` decides
which thruster each one drives.

| Output | J2 pin | PWM |
| --- | --- | --- |
| 1 | 7  | PWM7  |
| 2 | 12 | PWM3  |
| 3 | 15 | PWM15 |
| 4 | 33 | PWM8  |
| MOSFET enable | 13 | `mosfet_chip`, `mosfet_line` |
| Signal ground | 6 | |

The pins (`pwm_chips`, `pwm_channels`, `mosfet_*`) are `config/default.yaml`'s unless the vessel profile's
`thrusters:` sets them (the thrusters screen's p does); the operator file cannot move them. Each
`pwm_chips` glob must resolve to exactly one chip on the boat, and a controller the device-tree overlay
does not enable resolves to none: enable it in `host/odroid-m2-thruster-pwm-overlay.dts`. Run
`scripts/install_odroid_access.sh` once per ODROID, on the host, and again after the pins move. It
installs the pins (the vessel profile's, else the defaults') for a boot service that exports the PWM
channels and grants the container access to them.
Every entry point claims the MOSFET line first, sets neutral, and only then enables it. On exit it
turns the MOSFET off before PWM.

## Tests

```bash
python3 -m pytest -q        # pure Python, no ROS, a few seconds
```

The tests cover the core building blocks: the mixer, the velocity law, mux arbitration, the
e-stop and its clear, manual state, configuration loading, the ODROID PWM backend, `thrusters free`
and the thrusters screen's bar and profile save.
