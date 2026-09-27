# Boat commissioning dashboard

## Start safely

```sh
ros2 run mhseals_hardware boat_test --monitor-only
# Or allow explicitly confirmed physical actions:
ros2 run mhseals_hardware boat_test
```

Both start disarmed. No PWM/GPIO is opened, no command publisher is created,
and no sensors or bags are launched by default. The monitor-only flag also
blocks mapping/configuration changes and physical actions. Use a ROS-sourced
laptop/container or Odroid, with the correct ROS domain and network. A terminal
of at least 80×24 is recommended; the overview gains columns at 100 characters.

## Controls

| Key | Action |
| --- | --- |
| Tab / Left / Right | Switch overview, sensors, TF, mapping, results, logs |
| H | Arm at neutral after typing `ARM`, or disarm |
| M | Manual control; M/X/Esc returns, Space neutralizes |
| 1 / 2 / 3 | Surge / sway / yaw trials; each physical pulse needs confirmation |
| + / - | Adjust trial effort 5–50%; in manual mode adjust manual effort 0–100% |
| R | Disarm, edit FL/FR/RR/RL assignment, confirm atomic save; no pulses |
| I | Disarm, confirm, pulse and identify each output, persist complete mapping |
| B | Start/stop rosbag; later recordings get a new numbered directory |
| E | Export telemetry/configuration/trial summary as JSON |
| Space | Cancel an active timed phase; from dashboard, disarm |
| Q / Ctrl+C | Close, neutralize owned hardware and flush owned recordings |

Space in manual mode clears motion, but does not cut ESC power. Arming does
not start movement. With outputs armed, never edit wiring or pin assignments.
There is no physical emergency-stop guarantee: OS/process failure requires an
independent hardware cutoff. Neutral effort is not an active brake.

## Read the display

* **LIVE** means recent received data with basic quality checks, not merely a
  discovered topic. **WARN** includes no GPS fix, unknown/invalid GPS covariance,
  invalid orientation/velocity, wrong velocity frame, and bad timestamps.
  **STALE** is a receive timeout; **MISSING** means no sample has arrived.
* The motion panel separates requested **effort** from measured **m/s / rad/s**.
  Trends cover recent received samples, auto-scaled independently; they are not
  calibrated response plots. Stale values stay visible but are labeled stale.
* Thruster positions are canonical FL, FR, RR, RL. The diagram shows configured
  physical-output assignments and predicted mixer effort, **not motor feedback**.
  The mapping page reads exported PWM enable/period/duty values once per second.
  Unexported interfaces remain unavailable; monitoring never exports/enables them.
  Kernel readback is not proof of electrical waveform, ESC power, or rotation.
* TF checks directed connectivity and freshness of map→odom→base, plus chains
  to received GPS/IMU/lidar/camera frame IDs. It flags cycles and simultaneous
  competing parent/publisher records. Static transforms are retained; old static
  publishers may remain as conservative warnings until the monitor restarts.
  This is not tf2 interpolation testing or verification of physical mounting,
  axis conventions, timestamp synchronization, or localization accuracy.

Motion normally requires healthy odometry, GPS and IMU. `--require-tf` adds TF
chain health as a motion interlock. `--allow-missing-sensors` explicitly bypasses
sensor health for a secured, manually supervised thruster-only test; the screen
shows **SENSOR OVERRIDE**. TF gating, if enabled, remains active. Do not use an
override to claim odometry validation. Interlocks are checked throughout timed
and manual commands; failures cancel, send neutral and stop the owned driver.
Competing `/cmd_vel` publishers are rejected; an existing thruster driver also
prevents starting another. Direct external PWM writers cannot all be discovered
through ROS; run only one hardware controller.

## Sensors and portability

Default measurement is `/odom/local`, with `/odom/mavros` shown separately;
GPS `/gps/fix`, IMU `/imu/raw`, lidar `/points`, camera `/front_camera/rgb/image`.
Examples:

```sh
# Observe FCU odometry when the full EKF stack is not running:
ros2 run mhseals_hardware boat_test --monitor-only --odom-topic /odom/mavros
# Observe a compressed image topic available in this ROS domain:
ros2 run mhseals_hardware boat_test --monitor-only \
  --camera-topic /zed/zed_node/left/image_rect_color/compressed --camera-type compressed
# Explicitly start the real odometry stack and record:
ros2 run mhseals_hardware boat_test --start-stack --record --require-tf \
  --fcu-url serial:///dev/ttyACM0:57600
```

`--start-stack` uses mhseals_nav/odom.launch.py with `sim:=false`; do not use it
if another localization stack is running. `--optional-sensors` additionally
starts sensors.launch.py and requires `--start-stack`. Startup failures appear
in process logs. This dashboard does not bridge isolated Foxy/Jazzy domains:
a Foxy camera can be healthy yet correctly appear missing on the Jazzy Odroid.
Set `--map-frame`, `--odom-frame`, `--base-frame` for another robot's convention.

Shared hardware YAML and `--config`/`--save-config` remain the pin/mixer source
of truth; see [configuration.md](configuration.md). R changes channel mapping,
not pins or polarities. Full PWM paths are configurable, not tied to pwmchip
numbers. Mapping edits disarm and never automatically re-arm.

## Characterization and reports

Each axis runs positive then negative trials (`--repeats 1`, configurable 1–5).
Defaults: 25% effort, 5 s baseline, 3 s command, 5 s settling. CLI durations are
bounded to 60 s; do not increase them without sufficient clearance. Space/Q/Esc
interrupt the timed loop. There is no independently running command worker.

Results compare mean command-phase body velocity with pre-command drift, reporting
SIGN MATCH, SIGN MISMATCH, LOW RESPONSE, or INCONCLUSIVE. They include sample
counts, peak velocity, mean cross-axis velocity, gaps, and reference-frame pose
displacement when available. The 0.03 m/s or rad/s response threshold is a
heuristic; currents, tethering, GPS/IMU noise, and frame errors can change it.
Frame changes, missing data, or large gaps prevent a conclusive result. A sign
match does not validate speed calibration, dynamics, or Nav2 readiness.

Each completed trial saves `<bag-output>.report.json` atomically. E exports a
snapshot without requiring a bag. B records all ROS topics (watch disk space and
camera bandwidth); owned process logs append under `bags/logs/<session>/`.
Keep persistent configuration/bags in a mounted directory inside containers.

## Validation and maintenance

The existing PWM/mixer/configuration modules remain the actuator implementation.
`boat_monitor.py` owns bounded thread-safe telemetry and analysis without ROS;
`boat_dashboard.py` renders snapshots without side effects; `boat_test.py` adapts
ROS/processes and owns confirmed actions. Tests exercise these interfaces and
the actual production command loop with fake input/output, not duplicate logic.

```sh
python3 -m pip install pytest rich PyYAML
python3 -m pytest -q test
```

For the actual ROS subscription/TF/export adapter test, run
`test/boat_monitor_runtime_probe.py` with ROS sourced and `ROS_DOMAIN_ID=231`
**inside a network-isolated container without hardware devices**. It never
starts hardware or publishes `/cmd_vel`. Screen tests cover all views at 80×24
and 120×40. Local validation includes action cancellation/sensor-loss neutral,
layout checks, interactive menu/cancel/export checks, and ROS data freshness.

Hardware validation on 2026-09-27: four individual 1540 µs pulses produced no
visually reported rotation. A subsequent four-output 1600 µs test verified
enable=1, period=20,000,000 ns, selected duty=1,600,000 ns, other duties=1,500,000 ns,
and MOSFET GPIO high. All were neutral/disabled afterward. **Rotation from that
second sequence was not confirmed before boat access ended.** Do not interpret
the software tests or these readbacks as proof the thrusters physically work.
