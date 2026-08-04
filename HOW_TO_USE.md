# How to use the sim — quick navigation

A task-first cheat sheet: *"I want to know X"* → which script, and where to look.
(For install/setup see [README.md](README.md).)

---

## Step 0 — open it (do this once per MATLAB session)

1. Open **MATLAB**.
2. **File ▸ Open** → pick **`START.m`** in the repo folder → hit **Run** (green ▶ / F5).

`START.m` sets every path and prints the menu. You do **not** need to fiddle with the
"Current Folder". After that, just type a script name at the `>>` prompt and press Enter.

> If you ever see *"Undefined function"* — you skipped `START.m`. Run it first.

Everything a script produces (figures + CSVs) lands in the **`output/`** folder.

---

## "I want to know…" → run this

| I want to know… | Type this | What you get |
|---|---|---|
| **How efficient is the drivetrain, and what to fix** | `drivetrain_efficiency` | Printed battery→ground breakdown + a **7-tab figure** (see below) |
| **Which gear ratio is best** (efficiency + pack charge) | `gear_ratio_optimization` | Ranked ratio table + a dashboard + an operating-points map |
| **How quick is the car** (0–75 m / 0–100 kph) | `accel_model` | Accel times, tractive-effort and inertia plots |
| **Are the numbers trustworthy** | `verify_math` | 37 independent checks vs the source documents — all should say `[PASS]` |
| Endurance driveline torque spectrum | `fatigue_spectrum` | Load spectrum for fatigue |
| Accel driveline torque spectrum (the fatigue case) | `accel_fatigue` | Launch load spectrum |
| Braking / no-regen energy check | `brake_analysis` | Brake heat + energy lost with no regen |
| Physics model vs MEASURED efficiency | `efficiency_crosscheck` | Model-vs-telemetry comparison |

---

## Reading `drivetrain_efficiency` — the 7 tabs

Run `drivetrain_efficiency`, then click across the tabs in the figure window. Two groups:

**The hardware chain** (the fixed mechanical path: motor → spur gears → chain → diff → halfshafts → wheels):

| Tab | Shows |
|---|---|
| **Efficiency by stage** | One bar per stage (motor+inverter, spur, bearings, chain, diff, halfshaft). The shortest bar is the weakest link. Overall = all of them multiplied. |
| **Halfshaft angle sweep** | Overall efficiency vs halfshaft angle — how much straightening the shafts (12° → 0–5°) buys. |
| **Levers ranked** | Every fixable stage ranked by battery saved — **what to fix first.** |

**The motor** (how the motor is *used*, which is where the gearing/driving levers live):

| Tab | Shows |
|---|---|
| **Efficiency map** | Motor+inverter efficiency over every rpm×torque. Bright island = most efficient; red dots = where we actually run (part-load, off the island). |
| **Eff vs load** | Efficiency vs torque + a histogram of where we spend time. The point: ~31% of the lap sits in the low-torque **cliff**. |
| **Eff vs rpm** | Efficiency vs rpm — how it falls off if you rev past the sweet spot. |
| **Gear ratios on map** | The **same endurance laps under every gear ratio** — where each ratio puts the operating point (rpm & torque) vs the datasheet peak envelope. Lower ratio → more torque (better); higher → more rpm (toward redline). |

**The one-line takeaway the script prints:** ~62% battery→ground as-driven, ~68% ceiling
for the car as it sits; the motor+inverter is healthy (91% peak) — the losses are the
mechanical path plus running the motor part-loaded, **not** a worn motor.

---

## Reading `gear_ratio_optimization`

Prints a ranked table (efficiency, high-efficiency fraction, final SOC for each ratio
4.00–5.20) and opens two figure windows: a **dashboard** (battery validation, SOC vs
ratio, efficiency vs ratio) and an **operating-points map** (one panel per ratio).
Set the ratios it tests in `params_cfr26.m` (`p.gears_to_test`) — a single ratio or a
custom pair both work.

---

## The Simulink accel model (needs Simulink)

- **One ratio:** open **`accel_sim.slx`**, hit **Run** → prints the 0–75 m time, pops the
  distance/speed scope. Change the ratio by setting `G_ratio` in the workspace (defaults to 4.61).
- **All ratios:** double-click the green **RUN SWEEP** block, or type `sweep_accel_sim`.
- After changing `params_cfr26.m`, rebuild it: `build_accel_simulink`.

---

## Python tools (run in a terminal at the repo root, not MATLAB)

Only needed for data etc; the MATLAB analyses don't require them.

| Command | What it does |
|---|---|
| `python tools/emeter_unpack.py` | Unpack the FSAE competition e-meter archive + list what it logs |
| `python tools/emeter_benchmark.py` | Rank the field on endurance energy economy |
| `python tools/lap_feasibility.py` | Best-lap → 22-lap feasibility (energy / thermal / driver) |

---

## Change a number?

Every constant lives in **`params_cfr26.m`**, each with a comment saying where it came
from. Change it there once and every script picks it up. After a change, run
`verify_math` — it should still be all `[PASS]`.
