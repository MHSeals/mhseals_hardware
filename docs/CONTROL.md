# How the boat turns a command into thrust

The control stack is eight layers, **L0 to L7**, from the PWM pulse up to Nav2. Each layer has one
equation, one config key and one test that sets it, and each test's printout names its layer.
(Not to be confused with crane's **sim tiers T0/T1/T2**: abstract, dynamic, Unity. Tiers are how detailed
a simulation is; layers are parts of the boat's control.)

## Example first

```console
$ astro mix 0.5 0 0      # surge 0.5; here with an example C and r = 0.8
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

1. **C (L3).** The surge column of C is `[1, 0.04, -0.07]`: a `couple-surge` test found that pure surge
   also pushed the boat to port and turned it, so surge 0.5 also asks for sway +0.02 and yaw -0.035 to
   cancel that.
2. **M (L2).** FL's row is `[-1, +1, +1]`, so FL takes `-1·0.5 + 1·0.02 + 1·(-0.035) = -0.515`.
3. **e|e| (L1).** Thrust goes as `e|e|`, and reverse thrust is weaker by `r = 0.8`:
   `T = -0.515² · 0.8 = -0.212`. One axis alone gives back the same effort, `-0.515`. With several axes
   the thrusts add, and then `e = -sqrt(|T| / r)`.
4. **Desaturation (L2).** No `|e|` is over 1, so nothing is scaled.
5. **PWM (L0).** `1500 + 400 · (-0.515) = 1294 µs` on output 1.

`mix` prints L0 to L3 exactly as the driver computes them (both call `thruster_mixer.mix_stages`).

## The layers

The stack is a forward mix plus layered corrections, not an inverse allocation.

| Layer | Equation | Key | Test |
|---|---|---|---|
| L0 PWM | pulse = clip(1500 + 400·e, 1100, 1900) | fixed | – |
| L1 thruster law | thrust = e·\|e\|, × r when reversed | `reverse_thrust_ratio` | (bollard, later) |
| L2 geometry mix | u_ik = M_i·part_k; T_i = Σ_k u_ik\|u_ik\|; e = sgn·√\|T\|; uniform desaturation | `thruster_matrix`, `channel_map`, `reversed` | `wiring` |
| L3 decoupling | part_k = C[:,k]·cmd_k (C⁻ when cmd_k < 0) | `decoupling`, `decoupling_negative` | `couple-surge`, `-sway`, `-yaw` |
| L4 speed curve | e\|e\| = d1·v + d2·v\|v\| per signed axis | `characterization.speed_curves` | `speed-surge`, `-sway`, `-yaw` |
| L5 response | τ = t95 / 3 (first order) | `characterization.t95`, `motion_limits` | the same `speed-*` tests |
| L6 velocity loop | e = FF(L4⁻¹) + Kp·err + Ki·∫err, plus a drift observer | `controller.*` | `steps-surge`, `-sway`, `-yaw` (report only) |
| L7 Nav2 | MPPI limits from L4/L5, then critic weights | `characterization.nav2_limits` | `nav2`, then `trial` |

Before any of them, the `still` test measures the **conditions** (current and noise), which decide how high
up the ladder today's water can be trusted.

### The ladder

Run the tests bottom up; each one assumes the layers below it are set.

```bash
astro test still                       # conditions: ok or rough, and the highest layer worth trusting
astro down drive && astro wiring       # L2, once per wiring change, boat tied up; then astro up drive
astro test linear                      # L3 + L4/L5 for surge and sway: stops at each apply
astro test yaw                         # L3 + L4/L5 for yaw
astro down drive && astro up drive --with controller
astro test steps                       # L6: rise, overshoot, settling, steady-state error
astro test nav2 && astro apply nav2    # L7 limits; restart navigation, then astro trial
```

`astro test status` shows what is applied, measured, stale or pending, and the next step. A result goes
stale when a key below it changes (for example, a new C makes the speed curves stale).

## Each layer

### L0 PWM

- **Equation:** pulse = clip(1500 + 400·e, 1100, 1900) µs at 50 Hz; 1500 is stop.
- **Key / test:** none. Effort e is normalised to [-1, 1]; nothing here is tuned.

### L1 Thruster law

- **Equation:** T = e|e|; below zero, T × r.
- **Key:** `reverse_thrust_ratio` r, in [0.5, 1].
- **Test:** none yet. Keep `r = 1` until a bollard test measures it. A direction-dependent effect on one
  axis belongs in C⁻ instead. A `reversed` thruster has its weak side at positive effort.

### L2 Geometry mix

- **Equation:** for each axis k, thruster i alone would take `u_ik = M_i · part_k`. The thrusts add,
  `T_i = Σ_k u_ik|u_ik|` (× r below zero), and each thruster takes the effort that gives `T_i`. Adding
  thrust rather than effort keeps a diagonal straight. When any `|e|` is over 1, all four are divided by
  the largest, so the direction is kept and only the magnitude is lost.
- **Keys:** `thruster_matrix` M (rows FL, FR, RR, RL; columns surge, sway, yaw), then `reversed` (a
  thruster wired or mounted backwards) and `channel_map` (canonical FL, FR, RR, RL to outputs 1–4). The
  wiring keys live in the vessel profile's `thrusters:` section.
- **Test:** `astro wiring` (stop `drive` first, boat tied up): pulses each output and asks what moved.

```text
FL  -1  +1  +1
FR  -1  -1  -1
RR  +1  -1  +1
RL  +1  +1  -1
```

### L3 Decoupling

- **Equation:** part_k = C[:,k]·cmd_k. Each commanded axis takes its column of C, so "surge only" becomes
  the mix that actually moves the boat in surge only. C is 3×3, identity means no correction, entries
  within ±2, diagonal positive.
- **Keys:** `decoupling` (C) and `decoupling_negative` (C⁻, the column used when an axis is commanded
  below zero; `null` means same as C). Both change live.
- **Test:** `couple-<axis>` alternates + and − holds, sets each candidate column live, and stops when the
  cross response is inside the tolerance (surge, sway: 0.05 m/s and 0.015 rad/s; yaw: 0.10 and 0.10) or
  after 4 holds. `astro apply` writes the column and sets it on the driver:

```bash
ros2 param set /thruster_pwm_node decoupling "[1.0, 0.0, 0.0, 0.04, 1.0, 0.0, -0.07, 0.0, 1.0]"
```

### L4 Speed curve

- **Equation:** for each signed axis (`surge+`, `surge-`, … `yaw-`), the effort that holds a steady
  speed v is `e|e| = d1·v + d2·v|v|` (linear plus quadratic drag, in thrust units).
- **Key:** `characterization.speed_curves.<axis><sign>: {d1, d2}`, d1 ≥ 0.
- **Test:** `speed-<axis>` runs three levels each way (`large`: 0.3, 0.6, 0.9; `small`: 0.2, 0.35, 0.5)
  and fits d1, d2. The velocity controller's feed-forward is this curve inverted.

### L5 Response

- **Equation:** t95 is the rise time to 95 %; a first-order model has τ = t95 / 3.
- **Keys:** `characterization.t95` and `characterization.motion_limits` (`velocity`, `acceleration`,
  `deceleration`, each `{surge, sway, yaw}`).
- **Test:** the same `speed-*` run. The controller clips its target to `motion_limits.velocity` and
  refuses to start without it. `speed` results apply after a restart of `drive --with controller`.

### L6 Velocity loop

- **Equation:** `e = FF(target − bias − drift_body) + Kp·err + Ki·∫err`, clipped to [-1, 1]. FF is the
  L4 curve inverted; the integral stops on an axis pinned at ±1; the drift observer learns current and
  wind in the world frame while the boat holds still and rotates them into the body every cycle.
- **Keys:** `controller.kp`, `ki`, `integral_limit`, `drift_time_constant`, `drift_command_limit`,
  `drift_limit`, `drift_carry`, `proximity_distance`, `obstacle_timeout`, `odom_timeout`. The gains change
  live: `ros2 param set /calibrated_velocity_controller kp "[0.8, 0.8, 0.6]"`.
- **Test:** `steps-<axis>` (needs `drive --with controller`) steps to half the velocity limit and reports
  rise, overshoot, settling and steady-state error. It changes nothing: tune the gains live and rerun.

`calibrated_velocity_controller` turns Nav2's body velocity on `topics.auto` (`/cmd_vel`) into effort on
`topics.effort`, and drives only while the mux input is `auto` (otherwise it resets). Between L6 and L3
sits `command_mux`: duplicate > RC > chosen stop > keyboard > auto (ramped in from zero) > idle
([authority.md](authority.md)). Drift that changes with heading is not in C; L6 estimates it online.

### L7 Nav2

- **Equation:** MPPI's velocity and acceleration caps, each the smaller of what the boat showed (L5) and
  what its lag allows (stop within 1 m, ramps of 3 lags, a turn overshooting at most half the yaw
  tolerance).
- **Key:** `characterization.nav2_limits: {velocity: [vx, vy, wz], acceleration: [ax, ay, az]}`.
  mhseals_nav reads it at launch through this package's loader.
- **Test:** `nav2` needs no motion. Apply it and restart navigation. Critic weights are tuned after, with
  `astro trial` ([mhseals_nav: Tuning Nav2](https://github.com/MHSeals/mhseals_nav#tuning-nav2)).

## Where the values live

One state file holds everything measured and tuned (`~/.config/mhseals/hardware.yaml`), loaded over
`config/default.yaml` and the profile's wiring; only `astro apply` writes it. The layering and the live keys
are in the [README: Configuration](../README.md#configuration).
