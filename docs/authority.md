# Control authority, e-stop and ESC power

This file holds the design reasoning behind the safety paths in this package that is still in force:
which command drives the thrusters, how a human takes over, what stops the boat and what cuts ESC power.
Code comments cite it by section (for example `docs/authority.md 2.3`). The code is authoritative where
the two disagree. Fix this file in the same commit when that happens.

Host names never appear here. "The driver host" is whichever machine the vessel profile gives the
thruster outputs. "The TUI" is the mhseals_testing operator console on any station.

## 1. Two layers and the invariants

| Layer | Where | What it decides |
|---|---|---|
| Physical precedence | `command_mux` on the driver host, an arm child | which *input* reaches the driver: duplicate > rc > manual > hold > auto |
| Attribution | mhseals_testing (the arbiter in the session owner) | which *owner* is behind the input: rc, keyboard, hold, agent, task, nav, procedure |

Only the mux carries safety. The attribution layer only labels inputs and pauses or preempts them
cooperatively. Agents arm only through `boat_agent arm`, which shows a countdown that can be cancelled
(decision D-20261101-03). Nothing in this package arms.

Invariants:

1. **A human override always wins, at once, except over a duplicate mux.** The mux forwards the
   highest input in the same callback that receives it, with no timer wait. The one exception: while a
   second mux publishes the same output (`duplicate`), this mux sends zeros even under RC MANUAL or a
   stick takeover (2.2). Nothing between the mux and the driver consults the TUI, the
   arbiter or an agent. `pipeline_ms` in `~/state` is the measured time from receipt to the applied
   publish.
2. **Only the mux output drives.** The driver mixes `topics.applied` and nothing else. The only
   exception is a test pulse, which 2.4 allows only while the mux is idle.
3. **The release rule is written once**, in `Arbitration.request` (see 2.2).
4. **A hold is sticky.** It stays until it is released. The mux saves its holds (`hold_store`, in `command_mux.state_file`) and
   re-adopts them when it restarts, so a hold survives a mux restart, a TUI crash and a session takeover.
   When the hold is released, auto ramps back in from zero over `ramp`.

**This is not an independent failsafe.** RC drives only while the driver and the mux run. With no arm
owner, the keeper stops the children: the outputs go neutral and the MOSFET turns ESC power off. The
physical kill switch stays the last line. The FCU arm switch arms only the FCU. The thrusters do not
take their drive from the FCU outputs.

## 2. `command_mux`

### 2.1 Topics and publishing

Every topic name comes from the `topics:` section of `config/default.yaml`. The mux, the driver and the
velocity controller each read that section, so no node uses remaps.

| Dir | Key (default) | Notes |
|---|---|---|
| in | `rc` (`/mavros/rc/in`) | RCIn, sensor QoS |
| in | `manual` (`/cmd_vel/manual`) | effort from the TUI keys and Space, on any station |
| in | `effort` (`/cmd_vel_effort`) | the auto lane: the velocity controller's output, or an effort publisher |
| in | `~/request` | String JSON `{op: hold\|release, cause, by, token?}` |
| out | `applied` (`/cmd_vel/applied`) | what drove the boat: the driver mixes it, and analyses read it |
| out | `mux_state` (`/command_mux/state`) | String JSON, reliable, transient-local, depth 1 |

`topics.auto` (`/cmd_vel`) is the only SI velocity lane, and only the velocity controller reads it. The
mux sends output on every message of the active input, plus a keepalive at 25 Hz. The state is
published when it changes and again at 1 Hz. Ages, rates, `pipeline_ms` and the wall-clock `since_ns`
conversions do not count as a change (`command_mux.change_key`).

### 2.2 Inputs, holds and release

```
input ∈ {rc, manual, hold, auto, idle}            priority duplicate > rc > manual > hold > auto
duplicate ⇒ hold with zeros, ahead of rc: a second mux publishes the same output
rc      ⇔ rc valid ∧ (mode switch == MANUAL ∨ stick takeover latched)
manual  ⇔ a topics.manual message within command_timeout
hold    ⇔ any hold cause is set
auto    ⇔ a topics.effort message within command_timeout, ramped in over ramp; otherwise idle
```

- **RC valid:** a frame has at least 4 channels, is younger than `rc_max_age`, and has sticks and the
  mode channel inside 900-2100 µs. An empty channel list means there is no receiver.
- **Mode switch:** the mux decodes `mode_ch` with the FCU's own bands and `modes` (MODE1..6) from the
  raw PWM, never from `/mavros/state`, so a mode change from a GCS cannot grant RC authority.
- **Stick takeover:** in GUIDED, any of ch1, ch2 or ch4 held past `stick_takeover` µs for `stick_time`
  s latches rc. Once the sticks are centred for `stick_release` s, the latch becomes the `rc_stick`
  hold.
- **Hold causes:** `rc_switch` (the switch at HOLD or OTHER, a level that holds again after a release),
  `rc_lost` (RC was driving and its frames went invalid or stale), `rc_stop` (a falling edge on
  `stop_ch`), `rc_stick`, `keys` (manual input ended), and any cause a `~/request` names. `duplicate`
  is not a hold cause in this list: it outranks rc and cannot be released (it clears when the other mux
  goes).
  The TUI's Space places `keys`, and its E (revoke_agent) places its own cause.
- **Release:** moving the switch to GUIDED clears only the RC causes. A release with a token clears
  only the hold that token placed. A tokenless release (the TUI's U) clears every hold, except that it
  is refused with "mode switch is at MANUAL|HOLD|OTHER" while a valid frame puts the switch there. The
  TUI shows that refusal as "RC switch is ...".
- **Output:** hold and idle send zeros. Entering auto after a start, a hold, rc or manual ramps from zero
  over `ramp`, so a Nav2 output that kept running cannot lurch the boat when the hold is released.

### 2.3 `~/state` record

The contract is `schema == "mhseals.mux/1"` plus `input`. A consumer reads only `input`
(`command_arbitration.mux_input`):

| Consumer | Acts on | Unknown (another schema, no input, unreadable) |
|---|---|---|
| driver test pulses | allowed only while `idle` | refused |
| velocity controller | drives only while `auto`, and resets otherwise | reset |
| watchdogs | outranked by anything but `auto` and `idle` | outranked |

Every other field is for display, and none of them is a contract: `seq`, `t_ns`, `since_ns`, `cause`
(the reason the input became active: `rc_switch | rc_stick | stop_channel | manual_end | release |
manual | auto | idle`, or the raw hold cause: `rc_lost`, `duplicate` or a requested one; only `rc_stop`
and `keys` are renamed), `hold` (the oldest hold: `{by, cause, since_ns}`),
`holds` (each one with `has_token`, never the token), `rc` (valid, mode, pwm_mode, stick, rate_hz,
interval_p95_s, age_s, stop_edge), `manual`, `auto`, `rejected` (non-finite commands dropped),
`pipeline_ms`, `last_release` and `configuration`.

### 2.4 Test pulses

`~/test_pulses/held` (and the legacy `~/test_pulses`) reach the driver directly, not through the mux.
The driver accepts them **only while the mux input is `idle`**. They are refused in rc, manual, hold and
auto, and when the state is stale or unknown. So a bench pulse can outvote neither a human nor autonomy.
A live pulse ends at neutral as soon as the mux reports another input, or a mux frame that is not
neutral arrives. `~/output` reports each refusal and end with its reason.

## 3. E-stop

A cut turns the MOSFET off first, then sets the outputs to neutral, and latches for the life of the
driver (`estop.RemoteEstop`). Only a new arm starts a new driver, which powers the ESCs again with every
channel at neutral. While the cut is latched, the driver refuses drive and test pulses with
"e-stop latched (<cause>)", and `~/status` reports `estop.cause`.

| Cause | Trigger | Governed by |
|---|---|---|
| `rc-stop` | falling edge on `command_mux.stop_ch` (the mux edge rule itself) | `estop_cuts_power` |
| `rc-lost` | RC was in control (MANUAL or a stick takeover) and its frames went invalid or older than `rc_max_age` | `estop_on_rc_loss` |
| `operator` | the `~/estop` service (std_srvs/Trigger) | always |
| `stall` | a heartbeat flip gap over `HEARTBEAT_STALL` (section 4) | `mosfet_heartbeat` > 0 |

With `estop_cuts_power` true, the stop-switch edge both holds the mux and cuts ESC power. With it false,
the edge only holds the mux (cause `rc_stop`) and power stays on. A steady low on the stop switch does
nothing, so a transmitter left "disarmed" does not block autonomy.

The TUI stop is **Space**: neutral on `topics.manual` plus a `keys` hold. It holds; it does not cut
power. E is `revoke_agent`, not an e-stop. The remote arm link (mhseals_testing `ros/arm_link.py`) calls
`~/estop`.

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
- **Fitting:** with the circuit fitted and the rate left at 0, power drops about 0.22 s after arming.
  That fails safe, but set the rate when the hardware goes in. To verify, put a scope on the pin and the
  monostable output: the output falls within 0.25 s of `kill -9` or `kill -STOP`, stays low after
  `kill -CONT`, and shows no drops over 10 minutes under CPU stress.

## 5. Configuration lives one arm

The driver, the mux and the controller each read the arm snapshot once, at startup (a `-p` parameter
wins), and never watch the YAML. An edit applies at the next arm. The only live changes go through
SetParameters and are validated first: the driver's `decoupling` and `decoupling_negative`, the mux's
`rc_scale` and `ramp`, and the controller's gains. `command_timeout` is one top-level key that all three
stages read once per arm, so they never diverge.

## 6. On-boat checks

These steps can be proven only on hardware. Run them from the TUI or `boat_agent` with the playbook in
force. Each step names its atlas issue.

1. **E-stop latches (#89).** Props off, ESCs on the bench. Arm, then trigger in turn:
   `/thruster_pwm_node/estop`, the RC stop-channel edge, and transmitter-off while RC is in control
   (re-arm between them). Each one must latch with its cause (`operator`, `rc-stop`, `rc-lost`), cut ESC
   power (hardware.log "ESC power cut", the ESCs unpowered), refuse drive and pulses with "e-stop latched
   (<cause>)", and appear in events.jsonl.
2. **RC takeover and failsafe (#91).** Props off on the bench, or in water at low power, with the
   physical kill in reach. Bind the receiver and confirm that pre.rc passes. Then:
   - with the switch at GUIDED and an agent driving, hold ch1, ch2 or ch4 past `stick_takeover` for
     `stick_time`: the mux input becomes rc with cause `rc_stick`, and the takeover is journaled. rc with
     the switch at MANUAL (cause `rc_switch`) does not prove a takeover. Centre the sticks for
     `stick_release`: an `rc_stick` hold stays;
   - with the switch at HOLD or MANUAL, U refuses with "RC switch is ...";
   - transmitter-off while RC is in control latches rc-lost (with `estop_on_rc_loss` true, the default).
3. **Thrust reaches the ESCs (#88).** Read the close-out session's events.jsonl and hardware.log first.
   Then run Thruster set (i) at low power on each output: every prop spins, and the Rb column reads back
   the commanded µs.
4. **Velocity mode (#97), after step 3.** Boat in water, in open area; velocity tracking means nothing
   on a bench. Disarmed, run `boat_agent set command_mode=velocity`, then arm. Publish on `topics.auto`
   (`/cmd_vel`) at 10 Hz a surge step of 0.3 m/s (other axes 0) for 15 s, then 0. During the step the mux
   input must be `auto` (`~/state`), and `/odom/local` surge must settle within 5 s and then stay within
   0.1 m/s of the command, with no sustained sway or yaw. Manual keys skip the velocity controller
   (`topics.manual` enters the mux as effort), so they do not test this. This runs in the same nav smoke
   block as #96.
