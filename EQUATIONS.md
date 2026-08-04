# EQUATIONS.md — every model in this repo, written out

What this is: every equation the sim actually uses, with its symbols and the file it
lives in. If you're reviewing a number and want to know where it came from, start here,
then go read the file.

Ground rule this repo tries to keep: **no number appears without provenance.** Constants
live in `params_cfr26.m`, each tagged MEASURED / DATASHEET / GUESSESTIMATE. Equations
live here and in `lib/`. If something is unknown, the code says UNKNOWN or fails — it
does not quietly substitute a plausible-looking guess.

Where a model has a known bias or a caveat, it's called out inline. Read those.

---

## 1. Battery — coulomb counting

**Where:** `lib/run_open_loop.m`

State of charge by integrating current out of the cell:

```
SOC(k+1) = SOC(k) − I(k)·Δt(k) / Q
```

| Symbol | Meaning | Units |
|---|---|---|
| `SOC` | state of charge, 0 = empty, 1 = full | — |
| `I` | per-cell current, **positive = discharge** | A |
| `Δt` | timestep | s |
| `Q` | cell capacity `p.Q_cell` = 4.4 Ah × 3600 | A·s |

SOC is clamped to `[0, 1.2]`. **This is the only thing that sets SOC** — it does not
touch the R/C tables at all, which is why fixing the table indexing (below) moved the
modelled voltage but not a single SOC number.

## 2. Battery — 2-RC Thévenin terminal voltage

**Where:** `lib/run_open_loop.m`

Two RC branches: one fast sag, one slow. Each is a zero-order-hold discretisation of a
first-order RC:

```
V_RC1(k+1) = exp(−Δt/τ₁)·V_RC1(k) + R₁·(1 − exp(−Δt/τ₁))·I(k)
V_RC2(k+1) = exp(−Δt/τ₂)·V_RC2(k) + R₂·(1 − exp(−Δt/τ₂))·I(k)
        τᵢ = Rᵢ·Cᵢ

V_terminal = OCV(SOC) − I·R_i − V_RC1 − V_RC2
```

| Symbol | Meaning |
|---|---|
| `OCV(SOC)` | open-circuit (resting) voltage, interpolated from `p.rc.OCV_lookup` |
| `R_i` | instantaneous internal resistance (the immediate IR step) |
| `R₁,C₁` | fast RC branch — quick sag and recovery |
| `R₂,C₂` | slow RC branch — the long tail |

All from `p.rc.*`, fitted from our own HPPC cell test. Separate `_c` / `_d` table sets
for charging and discharging, because cells aren't symmetric.

**Table indexing.** The tables are indexed by **SOC** (index 1 = empty, index 11 = full),
on the shared grid `p.rc.SOC_lookupR = linspace(0,1,11)`. This used to be looked up at
`1−SOC` — backwards — while OCV was read at `SOC` off the same grid. Fixed. Settled
against the July 11 pack-voltage trace: on points where the tables actually do work (top
decile of |I_cell|) reading at SOC beats reading at 1−SOC by **6.3%** RMSE, and by
**9.5%** on the top 1%. Whole-trace RMSE barely discriminates because that log is
idle-dominated (median |I_cell| = 0.06 A).

**Trailing-zero artifact.** Every R and C table ends in a hard `0` at the top of the SOC
grid — the HPPC fit ran out of data there. A cell does not have zero resistance at 100%
SOC. The data is left exactly as the test produced it (`params_cfr26.m` is the record),
but every lookup drops that last grid point and clamps instead of interpolating into it.

## 3. Battery — Extended Kalman Filter (voltage-corrected SOC)

**Where:** `gear_ratio_optimization.m`, the EKF block

State `X = [SOC; V_R1; V_R2]`. Standard EKF, notation kept conventional on purpose.

```
Predict:   X⁻ = A·X + B·I
           A  = diag(1, exp(−Δt/τ₁), exp(−Δt/τ₂))
           B  = [ −Δt/Q ; R₁(1−exp(−Δt/τ₁)) ; R₂(1−exp(−Δt/τ₂)) ]
           P⁻ = A·P·Aᵀ + Q_noise

Measure:   Ut = OCV(SOC) − V_R1 − V_R2 − I·R_i        (predicted terminal voltage)
           H  = [ dOCV/dSOC , −1 , −1 ]               (measurement Jacobian)
           K  = P⁻·Hᵀ / (H·P⁻·Hᵀ + R_noise)           (Kalman gain)
           X  = X⁻ + K·(V_measured − Ut)
           P  = (I₃ − K·H)·P⁻
```

`dOCV/dSOC` comes from `gradient(p.rc.OCV_lookup, p.rc.SOC_lookupR)`. Coulomb counting
alone drifts; the voltage measurement pulls it back.

## 4. Motor — EMRAX 208 efficiency (copper + core loss)

**Where:** `lib/emrax208_efficiency.m`

```
I_rms      = T / Kt
P_copper   = 3 · I_rms² · R_phase
P_core     = a·rpm + b·rpm²
P_mech     = T · ω = T · rpm · 2π/60

η_motor    = P_mech / (P_mech + P_copper + P_core)
η_real     = η_motor · η_inverter
```

| Symbol | Params field | Meaning |
|---|---|---|
| `Kt` | `p.Nm_per_Arms` = 0.83 | torque per amp (DATASHEET) |
| `R_phase` | `p.R_phase` = 0.012 Ω | winding resistance per phase (DATASHEET) |
| `a`, `b` | `p.core_loss_a`, `p.core_loss_b` | core-loss fit: hysteresis ∝ rpm, eddy ∝ rpm² |
| `η_inverter` | `p.eta_inverter` = 0.95 | real-world haircut for inverter + switching/windage |

Result is clipped to `[0.05, 0.97]` to kill degenerate near-zero-load points.

**Known bias — constant Kt.** `Kt` is fixed, so current is assumed to scale linearly with
torque forever. A real EMRAX Kt *droops* as the iron saturates, so making a given torque
takes more current than this says. Copper loss goes as I², so this **underestimates
copper loss, worst at high torque**. A lower gear ratio demands more motor torque — so
the model is most optimistic exactly where the low-ratio recommendation lives. Treat
low-ratio efficiency gains as an upper bound. Fixing it needs a measured Kt-vs-current
curve for the 208, which we don't have.

**On `η_inverter`:** it was *chosen* to reconcile this physics model with the measured
pack-to-shaft number (§5). The agreement between them is a calibration, not a validation.

## 5. Measured pack-to-shaft efficiency (from telemetry)

**Where:** `drivetrain_efficiency.m` → `measured_pack_to_shaft()`;
`analysis/measured_efficiency_map.m` (same thing, binned into a map)

```
P_mech (out) = T_motor · ω_motor          (or axle torque × axle speed — ratio cancels)
P_elec (in)  = V_pack · I_pack

η_pack→shaft = Σ P_mech / Σ P_elec        over motoring points, energy-weighted
```

The gear ratio cancels, so this is a **pack → shaft** number: motor + inverter only, no
drivetrain hardware in it.

Motoring gate: `rpm > 500 & |T| > 5 Nm & P_pack > 500 W & 0.3 < η_inst < 1.0`.
Steady-state subset adds `movstd(rpm,11) < 40 & movstd(T,11) < 3`, which strips
transients — during acceleration, pack power also spins up rotor and wheel inertia, and
the instantaneous ratio wrongly books that as loss.

**Caveat on the word "measured":** the torque channel `PM100DX_torqueFeedback` is the
*inverter's own estimate* of torque, derived from measured current through its internal
motor model. It is not a torque transducer. So the mechanical side of this "measurement"
is partly model-derived and shares assumptions (notably Kt) with §4. A shaft torque
transducer on a dyno is what would make it independent.

## 6. Motoring / regen power convention

**Where:** `lib/motoring_regen_power.m`

Signed conversion between mechanical and electrical power:

```
P_elec = P_mech / η      when P_mech ≥ 0   (motoring — draw MORE than you use)
P_elec = P_mech · η      when P_mech < 0   (regen — return LESS than you make)
```

The sign flip matters: losses always work against you, in both directions.

## 7. Motor peak-torque envelope

**Where:** `lib/motor_peak_torque.m`

```
T_peak(rpm) = min( T_flat_cap , 9549 · P(rpm) / rpm )
```

Flat torque cap at low rpm (`p.T_flat_cap` = 150 Nm), power-limited above. `P(rpm)` is
interpolated from the datasheet curve `p.Prpm` / `p.Pkw`, clamped at redline. Below
50 rpm the power-derived torque is singular (P → 0), so the flat cap is used directly.
The 9549 converts kW and rpm to N·m.

## 8. Tyre — Pacejka peak longitudinal μ with load sensitivity

**Where:** `lib/tire_mu_x.m`

```
dfz = (Fz − FNOMIN) / FNOMIN
μ_x = LMUX · (PDX1 + PDX2 · dfz)
```

| Symbol | Params field | Meaning |
|---|---|---|
| `PDX1` | `p.tir.PDX1` = 2.1 | peak μ at nominal load |
| `PDX2` | `p.tir.PDX2` = −0.40981 | how μ falls as you squash the tyre harder |
| `FNOMIN` | `p.tir.FNOMIN` = 667 N | the load those numbers refer to |
| `LMUX` | `p.tir.LMUX` = 0.65 | scaling: test-rig belt → real pavement |

Works out to μ ≈ 1.37 at nominal load. Floored at 0.5 to guard degenerate extrapolation.

**Provenance warning:** Calspan never ran a *longitudinal* sweep on this tyre (ours was
too small). The lateral numbers are real test data; **these forward-grip numbers are a
well-dressed estimate.** See §10 for why the accel model's grip sensitivity is currently
unverifiable.

## 9. Acceleration — inertia-based ODE over motor speed

**Where:** `accel_model.m` → `accel_run()`

Integrates motor speed, counting the rotational inertia you have to spin up as well as
the car you have to push (WR-217e / FSAE top-speed method):

```
dω/dt = [ T_use − n·r·(b·ω² + C) − T_F ] / I_den

  n     = 1/G                                        (inverse gear ratio)
  I_den = I_rotor + I_driveline + n_w·I_wheel·n²  +  n·r·(m·n·r)
          └───────── rotating inertia ──────────┘   └ car mass reflected ┘
  b     = ½·ρ·r²·n²·(CdA + Crr·ClA)                  (speed-squared resistance)
  C     = m·g·Crr                                    (rolling resistance)
  T_F   = p.T_F = 1.5 N·m                            (driveline friction)
```

Then `v = r·n·ω`, and distance integrates from there. `I_wheel = k²·m_wheel·r²` with
`k = p.kFactor = 0.60` (GUESSESTIMATE — how far out the wheel's mass sits).

Torque actually used, with weight transfer converged in an inner loop:

```
Fz_rear = m·g·rear_static + m·a_x·h_cg/L_wb + F_down·rear_aero
F_tract = μ_x(Fz_rear/2) · Fz_rear
T_use   = min( T_peak(rpm)·η_drivetrain , F_tract·r·n/η_drivetrain )
```

> ### ⚠ KNOWN BUG — the traction cap's units are inconsistent
> `T_M = T_peak·η` already has `η_drivetrain` applied, so it's post-loss shaft torque.
> The cap on the right then divides by `η` **again**. Both are motor-shaft torques, so
> the cap should be `F_tract·r·n` with no `/η`. As written the cap is inflated by
> `1/η ≈ 1.26×` and **never binds**: 0 of ~3726 integration steps to 75 m hit it. In
> consistent units, ~1900 of ~3720 steps *are* traction-capped and 0–75 m at 4.61:1 goes
> 4.669 → 4.714 s. `verify_math` §8 independently computes a traction-limited launch, so
> the two disagree — same bug showing up twice.
>
> **Consequence:** any claim that "the launch isn't traction-limited" or "grip doesn't
> affect accel" is an **artifact of this bug, not a result**. The grip sensitivity of
> 0–75 m is **unverified in both directions** — this is OPEN pending launch/TC data.
> Left unfixed deliberately so no number moves until that data lands.

**Also unmeasured — wheel radius.** `p.r_wheel` = 0.2286 m is the **free** radius; a
loaded tyre squishes ~5% smaller and nobody has measured ours. `r` sets both rpm→speed
and the tractive force arm, so ~5% on radius is ~5% on force — a bigger lever than the
halfshaft angle. At 0.95×, 0–75 m goes 4.669 → 4.579 s, closing about a third of the
0.27 s sim-vs-measured gap. `accel_model.m` prints this sensitivity; the value is not
changed.

A simpler traction-limited point-mass version lives in `lib/accel_075m.m` (used by the
gear study's quick sweep), and top speed marches the same force balance in
`lib/top_speed.m` until drive force can no longer beat drag + rolling.

## 10. Halfshaft / CV-joint loss

**Where:** `params_cfr26.m` (constants), `drivetrain_efficiency.m`, `accel_model.m`

A CV joint loses power in proportion to the angle it works at. Two joints per shaft:

```
η_shaft(β) = 1 − 2·k_loss·sin(β)
```

Lap-weighted, because a lap isn't all straight:

```
η_lap = f_straight·η_shaft(β) + (1 − f_straight)·η_shaft(β + β_corner)
```

| Symbol | Params field | Value | Note |
|---|---|---|---|
| `k_loss` | `p.hs_kloss` | 0.090 | friction-geometry coefficient — see warning below |
| `β` | `p.hs_angle_deg` | 12° | **MEASURED** from suspension CAD, static ride height, straight ahead |
| `β_corner` | `p.hs_corner_deg` | 8° | extra articulation in a loaded corner |
| `f_straight` | `p.hs_frac_straight` | 0.724 | fraction of an endurance lap near the static angle |

At 12°: `η_lap = 0.9559`. Straightening gives `η_lap(5°) = 0.9775`, `η_lap(0°) = 0.9931`.
As an efficiency **gain** that's **+2.25%** (12→5°) and **+3.89%** (12→0°); as **battery
saved** per lap it's +2.20% / +3.74% (+111 / +189 Wh). Two different quantities — don't
mix them up in a slide.

> **⚠ `k_loss` has no independent source.** It is **back-fitted** to the CFR26 DT memo's
> own two *assumed* points (0.99 @ 3°, 0.94 @ 20°). Check:
> `1 − 2(0.09)sin3° = 0.9906`, `1 − 2(0.09)sin20° = 0.9384`. It reproduces them because
> it was fitted to them — a **round-trip, not a cross-check**. Published CV-joint loss
> figures are generally lower, so this model is probably **pessimistic** about the
> current 12° shafts, which biases the repackaging case in its own favour. A back-to-back
> dyno pull or a coastdown at two angles would settle it.

> **⚠ Bump/droop is not modelled.** β swings through suspension travel, and
> `1 − 2k·sin β` is nonlinear in β, so the travel-averaged loss ≠ the loss at the average
> angle. This evaluates at the static angle plus a flat cornering term; it does not
> integrate over travel. Direction of the error is not obvious.

## 11. Mechanical drivetrain stack

**Where:** `params_cfr26.m` (`p.eta_drivetrain`), `drivetrain_efficiency.m`

Everything multiplies:

```
η_drivetrain = η_spur · η_bearings · η_chain · η_diff · η_halfshaft(β)
             = 0.98 · 0.95 · 0.97 · 0.92 · 0.9559
             = 0.794
```

and the full battery-to-ground number stacks the electrical end on top:

```
η_overall = η_pack→shaft(MEASURED) · η_spur · η_bearings · η_chain · η_diff · η_halfshaft(β)
```

The four stage values are the CFR26 DT memo's **assumptions**, not measurements on our
car — each is dyno- or coastdown-measurable and none has been measured yet.
`verify_math` §11 gates that this product still equals `p.eta_drivetrain`, so a stage
edit can't silently disagree with the single number the rest of the sim uses.

A stage going from current `c` to best `b` lifts overall efficiency by `b/c` and saves
`1 − c/b` of the pack.

## 12. Fatigue load spectrum (torque binning)

**Where:** `fatigue_spectrum.m` (endurance), `accel_fatigue.m` (launches)

Time-at-torque histogram, normalised over *loaded* time:

```
bin_time(k) = Σ Δt   for all samples with  edges(k) ≤ T < edges(k+1)
spectrum(k) = bin_time(k) / Σ bin_time
```

Bin edges match the team's existing `Fatigue Load Cases.xlsx` so the output pastes
straight into the fatigue tool, extended to 150 Nm (motor peak) for endurance and to
160 Nm in `accel_fatigue.m` to catch command overshoot. Near-zero-torque coasting (below
the first 0.1 N·m edge) is excluded from the normalisation and reported separately —
otherwise idle time would dilute every bin and make the driveline look under-loaded.

The two scripts are complementary: endurance fills the low bins, launches fill the
140–160 N·m bins that endurance leaves empty.

---

## Where the honesty tags live

`params_cfr26.m` is the single source of truth for every constant, and each one carries
its provenance on the same line:

- **MEASURED** — we physically measured it. Trust it.
- **DATASHEET** — the manufacturer says so.
- **GUESSESTIMATE** — educated guess, mostly estimate.

Things currently known to be assumptions rather than measurements, collected in one
place so nobody has to go digging:

| Thing | Status | What would settle it |
|---|---|---|
| Spur / bearings / chain / diff stage efficiencies | memo ASSUMPTIONS | coastdown or back-to-back dyno |
| `p.hs_kloss` = 0.090 | back-fitted to the memo's own assumed points | dyno/coastdown at two halfshaft angles |
| `p.r_wheel` loaded radius | free radius used; loaded ~5% smaller, unmeasured | measure it under load |
| `p.tir.PDX1` / `PDX2` longitudinal grip | DERIVED — no longitudinal tyre test exists | a longitudinal tyre test |
| `p.eta_inverter` = 0.95 | calibrated to our own telemetry, one session | deliberate steady-state / dyno run |
| Constant `Kt` in the motor model | known bias, no saturation model | measured Kt-vs-current curve |
| accel traction cap | **unit bug, cap never binds** | launch/TC data — then fix the units |
