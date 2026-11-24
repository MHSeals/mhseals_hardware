# How the boat turns a command into thrust

## Example first

```console
$ ros2 run mhseals_hardware mix 0.5 0 0      # surge 0.5, with a measured C and r = 0.8
L3 decoupling  part_k = C[:,k] * cmd_k  (C- column when cmd_k < 0)
  axis    cmd     column  part_k (surge, sway, yaw)
  surge  +0.500  C      +0.500  +0.020  -0.035
  sway   +0.000  C      +0.000  +0.000  +0.000
  yaw    +0.000  C      +0.000  +0.000  +0.000

L2 geometry mix  u_ik = M_i . part_k;  T_i = sum_k u_ik|u_ik| (x r when < 0)
L1 thruster law  e_i = sgn(T_i) sqrt|T_i| (/ r when < 0), r = 0.8
  thr   M row (surge sway yaw)     u_surge  u_sway   u_yaw      T_i     e_i
  FL  -1.000 +1.000 +1.000     -0.515  +0.000  +0.000   -0.212  -0.515
  FR  -1.000 -1.000 -1.000     -0.485  +0.000  +0.000   -0.188  -0.485
  RR  +1.000 -1.000 +1.000     +0.445  +0.000  +0.000   +0.198  +0.445
  RL  +1.000 +1.000 -1.000     +0.555  +0.000  +0.000   +0.308  +0.555
  desaturation: peak |e| 1.000 -> every e_i / 1.000

L0 PWM  reversed flips e; pulse = clip(1500 + 400 e, 1100, 1900); channel_map routes
  thr   e_i     reversed  e_out   pulse  -> output
  FL   -0.515   False    -0.515    1294  -> 1
  ...
outputs 1-4 (us): 1294 1306 1678 1722
```

Read it top down:

1. **C.** The surge column of C is `[1, 0.04, -0.07]`: the `couple-surge` test found that pure
   surge also pushed the boat to port and turned it left, so asking for surge 0.5 also asks for
   sway +0.02 and yaw -0.035 to cancel that.
2. **M.** FL's row is `[-1, +1, +1]`, so FL takes `-1·0.5 + 1·0.02 + 1·(-0.035) = -0.515`.
3. **e|e|.** Thrust goes as `e|e|`, and reverse thrust is weaker by `r = 0.8`:
   `T = -0.515² · 0.8 = -0.212`. One axis alone gives back the same effort, `-0.515`.
   With several axes the thrusts add, and then `e = -sqrt(|T| / r)`.
4. **Desaturation.** No `|e|` exceeds 1, so nothing is scaled.
5. **PWM.** `1500 + 400 · (-0.515) = 1294 µs` on output 1.

## The layers

The stack is a forward mix plus layered corrections, not an inverse allocation. Each layer
has one config key and one test that sets it. Each test's printout names its layer.

| Layer | Equation (code) | Key | Set by |
|---|---|---|---|
| L0 PWM | pulse = clip(1500 + 400·e, 1100, 1900) (`thruster_mixer.effort_to_pulse`) | fixed | – |
| L1 thruster law | thrust ∝ e·\|e\|, reverse × r (`thruster_mixer.thrust_of`, `effort_of`) | `reverse_thrust_ratio` | (bollard, later) |
| L2 geometry mix | T_i = Σ_k M_ik·part_k, then e = sgn·√\|T\|, then uniform desaturation (`thruster_mixer._mix`) | `thruster_matrix` M, `channel_map`, `reversed` | `wiring` |
| L3 decoupling | part_k = C[:,k]·cmd_k (C⁻ when cmd_k < 0) (`thruster_mixer.decouple`) | `decoupling`, `decoupling_negative` | `couple` |
| L4 speed curve | e\|e\| = d1·v + d2·v\|v\| per signed axis | `characterization.speed_curves` | `speed` |
| L5 response | t95; first-order τ = t95/3 | `characterization.t95`, `motion_limits` | `speed` |
| L6 velocity loop | e = FF(L4⁻¹) + Kp·err + Ki·∫err, plus a world-frame drift observer (`velocity_law.VelocityLaw.step`) | `controller.*` | `steps` (report) |
| L7 Nav2 | MPPI limits from L4/L5, then critic weights | `characterization.nav2_limits` | `astro test nav2`, `trial` |

Drift that changes with heading is not in the matrix. L6 estimates it online, in the world frame.

`mix` prints L0 to L3 exactly as the driver computes them. Both call
`thruster_mixer.mix_stages`.

### L0 PWM

The ESCs take 1100–1900 µs at 50 Hz, with 1500 as stop. Effort is normalised to [-1, 1].
Nothing here is tuned.

### L1 Thruster law

A propeller's thrust grows with the square of its speed, so the mixer works in
`T = e|e|`. Below zero it multiplies by `r` (`reverse_thrust_ratio`, in [0.5, 1]). Keep `r = 1`
until a bollard test measures it. A direction-dependent effect on one axis belongs in C⁻
instead. The ratio follows the propeller, so a `reversed` thruster has its weak side at
positive canonical effort.

### L2 Geometry mix

`thruster_matrix` M has rows FL, FR, RR, RL and columns surge, sway, yaw:

```text
FL  -1  +1  +1
FR  -1  -1  -1
RR  +1  -1  +1
RL  +1  +1  -1
```

For each axis k, thruster i would take `u_ik = M_i · part_k` alone. The thrusts of those
efforts add (`T_i = Σ_k u_ik|u_ik|`, × r below zero), and each thruster takes the effort that
gives `T_i`. Adding thrust rather than effort is what keeps a diagonal straight. When any `|e|`
is over 1, all four are divided by the largest. Every thrust then shrinks by the same factor,
so the direction is kept and only the magnitude is lost.

After that, `reversed` flips a thruster that is wired or mounted backwards, and `channel_map`
routes canonical FL, FR, RR, RL to physical outputs 1–4. The `wiring` test sets both
(`thruster_identify`, with the boat tied up). A vessel's wiring lives in its profile's
`thrusters:` section.

### L3 Decoupling

C is a 3×3 matrix (rows and columns surge, sway, yaw; identity means no correction). Each
commanded axis takes its column of C, so "surge only" becomes the mix that actually moves
the boat in surge only. A coupling that differs by direction (drag at a leading corner, for
example) needs a column per sign: `decoupling_negative` (C⁻) gives the column for an axis
commanded below zero, and `null` means it is the same as C. Entries stay within ±2 and the
diagonal stays positive.

The `couple-<axis>` test alternates + and − holds and refines that axis's column of C and C⁻.
`astro apply` writes the column and sets it live on the driver:

```bash
ros2 param set /thruster_pwm_node decoupling "[1.0, 0.0, 0.0, 0.04, 1.0, 0.0, -0.07, 0.0, 1.0]"
```

### L4 Speed curve

For each signed axis (`surge+`, `surge-`, … `yaw-`), the effort that holds a steady speed v is

```text
e|e| = d1·v + d2·v|v|        (linear + quadratic drag, in thrust units)
```

The `speed-<axis>` test runs three levels each way and fits d1 ≥ 0 and d2 into
`characterization.speed_curves.<axis><sign>: {d1, d2}`. The velocity controller's feed-forward
is the inverse of this curve.

### L5 Response

The same `speed` test measures the rise time to 95 % (t95). A first-order model has
τ = t95/3. From these it derives `characterization.t95` and `characterization.motion_limits`
(`velocity`, `acceleration`, `deceleration`: `{surge, sway, yaw}`). The controller clips its
target to `motion_limits.velocity` and refuses to start without those limits.

### L6 Velocity loop

`calibrated_velocity_controller` (`drive.launch.py controller:=true`) turns Nav2's body
velocity on `topics.auto` into effort on `topics.effort`:

```text
e = FF(target − bias − drift_body) + Kp·err + Ki·∫err      clipped to [-1, 1]
```

- FF is the L4 curve inverted.
- The integral stops on an axis that is pinned at ±1.
- The drift observer learns current and wind in the world frame while the boat holds still,
  and rotates them into the body every cycle.
- It drives only while the mux input is `auto`. Otherwise it resets.

Between L6 and L3 sits `command_mux`. Its priority is duplicate > RC > chosen stop (HOLD switch,
stop channel) > keyboard > auto (ramped in from zero) > idle. A stale lane falls through to the
next one (docs/authority.md).

The gains are `controller.kp`, `ki`, `integral_limit`, `drift_time_constant`,
`drift_command_limit`, `drift_limit`, `drift_carry`, `proximity_distance`, `obstacle_timeout`
and `odom_timeout`. They change live:

```bash
ros2 param set /calibrated_velocity_controller kp "[0.8, 0.8, 0.6]"
```

The `steps-<axis>` test reports rise, overshoot, settling and steady-state error. It changes
nothing.

### L7 Nav2

The `nav2` test needs no motion. It derives the MPPI caps from L4 and L5 and writes
`characterization.nav2_limits: {velocity: [vx, vy, wz], acceleration: [ax, ay, az]}`.
mhseals_nav reads them through this package's loader. Critic weights are tuned later, with
`trial`.

## Where the values live

Each layer is loaded over the one before it, and the driver, mux, controller, `mix` and
`config_check` all load them the same way:

1. `config/default.yaml`: the structure and defaults, kept in git. `config/schema.yaml`
   documents each key.
2. The vessel profile's `thrusters:` section: only `channel_map` and `reversed`.
3. `~/.config/mhseals/hardware.yaml` (or `$MHSEALS_HARDWARE_CONFIG`, or `config_file:=`): the
   measured and tuned state. Only `astro apply` writes it, and then runs `config_check` on it.

Nodes read the config once, when they start. The only live keys are the driver's
`decoupling` and `decoupling_negative`, the controller's gains, and the mux's `rc_scale` and
`ramp`.
