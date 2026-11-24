# Control authority, e-stop and ESC power

This file holds the design reasoning behind the safety paths in this package that is still in force:
which command drives the thrusters, how a human takes over, what stops the boat and what cuts ESC power.
Code comments cite it by section (for example `docs/authority.md 2.3`). The code is authoritative where
the two disagree. Fix this file in the same commit when that happens.

Host names never appear here. "The driver host" is whichever machine the vessel profile gives the
thruster outputs. `drive.launch.py` starts the driver and the mux there (and the velocity
controller with `controller:=true`); docs/CONTROL.md explains the control model.

## 1. Invariants

1. **A lost input goes neutral, and nothing stays locked afterwards.** A lane that goes stale or
   invalid falls through to the next one (within `command_timeout`, 0.5 s). Whoever next means to
   move the boat (keys, RC, a controller, a test) moves it at once, with no unlock step.
2. **Only stops someone chooses hold**, and each clears in an obvious way (2.2, 3).
3. **A human override wins at once, except over a duplicate mux.** The mux forwards the highest input
   in the same callback that receives it. While a second mux publishes the same output (`duplicate`),
   this mux sends zeros even under RC.
4. **Only the mux output drives.** The driver mixes `topics.applied` and nothing else.

**This is not an independent failsafe.** RC drives only while the driver and the mux run. When the
stack stops, the outputs go neutral and the MOSFET turns ESC power off. The physical kill switch stays
the last line. The FCU arm switch arms only the FCU. The thrusters do not take their drive from the FCU
outputs.

## 2. `command_mux`

### 2.1 Topics and publishing

Every topic name comes from the `topics:` section of `config/default.yaml`. The mux, the driver and the
velocity controller each read that section, so no node uses remaps.

| Dir | Key (default) | Notes |
|---|---|---|
| in | `rc` (`/mavros/rc/in`) | RCIn, sensor QoS |
| in | `manual` (`/cmd_vel/manual`) | effort from `keyboard_control` |
| in | `effort` (`/cmd_vel_effort`) | the auto lane: the velocity controller's output, or an effort publisher |
| out | `applied` (`/cmd_vel/applied`) | what drove the boat: the driver mixes it, and analyses read it |
| out | `mux_state` (`/command_mux/state`) | String JSON, reliable, transient-local, depth 1 |

`topics.auto` (`/cmd_vel`) is the only SI velocity lane, and only the velocity controller reads it. The
mux sends output on every message of the active input, plus a keepalive at 25 Hz. The state is
published when it changes and again at 1 Hz. Ages, rates, `pipeline_ms` and the wall-clock `since_ns`
conversions do not count as a change (`command_mux.change_key`).

### 2.2 Inputs and stops

```
input ∈ {rc, manual, hold, auto, idle}       priority duplicate > rc > manual > stop > auto > idle
duplicate ⇒ zeros (input hold): a second mux publishes the same output; clears when it goes
rc      ⇔ rc valid ∧ (mode switch == MANUAL ∨ stick takeover latched)
manual  ⇔ a topics.manual message within command_timeout
stop    ⇔ (rc valid ∧ mode switch at HOLD or an unknown mode) ∨ stop channel latched   (input hold)
auto    ⇔ a topics.effort message within command_timeout, ramped in over ramp; otherwise idle
```

- **RC valid:** a frame has at least 4 channels, is younger than `rc_max_age`, and has sticks and the
  mode channel inside 900-2100 µs. An empty channel list means there is no receiver.
- **Mode switch:** the mux decodes `mode_ch` with the FCU's own bands and `modes` (MODE1..6) from the
  raw PWM, never from `/mavros/state`, so a mode change from a GCS cannot grant RC authority.
- **Stick takeover:** in GUIDED, any of ch1, ch2 or ch4 held past `stick_takeover` µs for `stick_time`
  s latches rc. Once the sticks are centred for `stick_release` s, the takeover ends and the next lane
  drives.
- **HOLD switch:** a level. It holds while the switch is there and frees when it moves out. When RC is
  lost, it holds nothing.
- **Stop channel:** a falling edge on `stop_ch` holds until that switch goes back up. With
  `estop_cuts_power` it also latches the driver's e-stop, which only a restart of `drive.launch` clears
  (3).
- **Output:** a stop and idle send zeros. Entering auto after a start, a stop, rc or manual ramps from
  zero over `ramp`, so a controller that kept publishing cannot lurch the boat.

### 2.3 `~/state` record

The contract is `schema == "mhseals.mux/1"` plus `input`. A consumer reads only `input`
(`command_arbitration.mux_input`):

| Consumer | Acts on | Unknown (another schema, no input, unreadable) |
|---|---|---|
| velocity controller | drives only while `auto`, and resets otherwise | reset |

Every other field is for display, and none of them is a contract: `seq`, `t_ns`, `since_ns`, `cause`
(why the input became active: `rc_switch | rc_stick | manual | auto | idle`, or the stop:
`duplicate | stop_channel | rc_switch`), `hold` (the first stop in force: `{by, cause, since_ns}`),
`holds` (every stop in force: `{cause, since_ns}`), `rc` (valid, mode, pwm_mode, stick, rate_hz,
interval_p95_s, age_s, stop_edge), `manual`, `auto`, `rejected` (non-finite commands dropped),
`pipeline_ms` and `configuration`.

## 3. E-stop

A cut turns the MOSFET off first, then sets the outputs to neutral, and latches for the life of the
driver (`estop.RemoteEstop`). Only a restart of the driver powers the ESCs again, with every
channel at neutral. While the cut is latched, the driver ignores drive commands, and `~/status` reports `estop.cause`.

| Cause | Trigger | Governed by |
|---|---|---|
| `rc-stop` | falling edge on `command_mux.stop_ch` (the mux edge rule itself) | `estop_cuts_power` |
| `operator` | the `~/estop` service (std_srvs/Trigger) | always |
| `stall` | a heartbeat flip gap over `HEARTBEAT_STALL` (section 4) | `mosfet_heartbeat` > 0 |

With `estop_cuts_power` true, the stop-switch edge both holds the mux and cuts ESC power. With it false,
the edge only holds the mux (cause `stop_channel`, until the switch goes back up) and power stays on. A
steady low on the stop switch is no edge, so a transmitter left "disarmed" does not block autonomy.

Losing RC never cuts: the mux falls through and the command timeout brings neutral.

Clearing a latched cut: `rc-stop`, `operator` and `stall` all clear only by restarting `drive.launch`
(`astro estop` trips `operator`).

## 4. MOSFET heartbeat

Goal: ESC power drops by itself within about 0.2-0.5 s when the driver stops running (crash, SIGKILL,
OOM, a hang or a kernel freeze), not only when it exits cleanly. Without the heartbeat circuit, a hard
death leaves the enable pin at its last level.

- **Circuit:** the enable line goes into a retriggerable monostable (a 74LVC1G123 at 3.3 V, about 0.22 s
  pulse width from 100 kΩ and 2.2 µF), whose output drives the MOSFET module. Pull-downs on its input
  and on the module input mean that a released, unplugged or unpowered stage reads as off. Use the gpiod
  line, never a hardware PWM pin, because kernel PWM keeps running after the process dies.
- **Driver:** with `mosfet_heartbeat` at N Hz (0 means a steady level, for no circuit), a ROS timer on
  the driver's executor flips the line every 1/(2N) s while it is powered. A hung node therefore stops
  the edges. A gap over `HEARTBEAT_STALL` (0.15 s) latches a `stall` cut, so a node that resumes cannot
  power the ESCs again. An e-stop and shutdown hold a steady inactive level.
- **Order at open:** the gpiod request comes first (the only exclusive resource), then the channels go
  to neutral, then enable, then the beats.
- **Not covered:** the monostable or the MOSFET failing high, and wiring after the monostable. The
  command watchdog and the holds remain.
- **Fitting:** with the circuit fitted and the rate left at 0, power drops about 0.22 s after the driver starts.
  That fails safe, but set the rate when the hardware goes in. To verify, put a scope on the pin and the
  monostable output: the output falls within 0.25 s of `kill -9` or `kill -STOP`, stays low after
  `kill -CONT`, and shows no drops over 10 minutes under CPU stress.

## 5. Configuration is read once

The driver, the mux and the controller each read the config once, at startup (a `-p` parameter
wins), and never watch the YAML. An edit applies at the next start. The only live changes go through
SetParameters (`ros2 param set`) and are validated first: the driver's `decoupling` and
`decoupling_negative`, the mux's `rc_scale` and `ramp`, and the controller's gains. `command_timeout`
is one top-level key that all three stages read once, so they never diverge.

## 6. On-boat checks

These steps can be proven only on hardware.

1. **E-stop latches.** Props off, ESCs on the bench. Start `drive`, then trigger in turn:
   `ros2 service call /thruster_pwm_node/estop std_srvs/srv/Trigger`, the RC stop-channel edge, and
   transmitter-off while RC is in control (restart between them). Each one must latch with its cause
   (`operator`, `rc-stop`, in `/thruster_pwm_node/status`), cut ESC power, and leave the outputs at
   neutral. Transmitter-off while RC is in control must NOT latch: the outputs go neutral within 0.5 s.
2. **RC takeover and failsafe.** Props off on the bench, or in water at low power, with the physical
   kill in reach:
   - with the switch at GUIDED and effort on `topics.effort`, hold ch1, ch2 or ch4 past
     `stick_takeover` for `stick_time`: the mux input becomes rc with cause `rc_stick`. Centre the
     sticks for `stick_release`: the effort lane drives again;
   - with the switch at HOLD, the input is `hold`; moved back to GUIDED, effort drives again;
   - transmitter-off while RC is in control: neutral within 0.5 s, no latch.
3. **Thrust reaches the ESCs.** `thruster_test` at low power on each output: every prop spins, and
   `/thrusters/pwm` reads back the commanded µs while `drive` runs.
4. **Velocity mode, after step 3.** Boat in water, in open area. `drive controller:=true`, then publish
   on `topics.auto` (`/cmd_vel`) at 10 Hz a surge step of 0.3 m/s for 15 s, then 0. The mux input must
   be `auto`, and `/odom/local` surge must settle within 5 s and stay within 0.1 m/s of the command.
