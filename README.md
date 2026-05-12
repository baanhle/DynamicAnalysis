# Train–Track–Bridge 2D Python Version

**Train–Track–Bridge 2D Dynamic Interaction Simulation**

License: GNU General Public License v3.0

---

## Overview

TTB-2D simulates the dynamic response of a railway bridge under moving train loads using the Finite Element Method (FEM). It solves the fully coupled Vehicle–Track–Bridge Interaction (VBI) problem in the time domain using the Newmark-β integration method.

**Key capabilities:**
- Euler-Bernoulli beam FEM for bridge and rail
- 6-DOF vehicle model (body + 2 bogies + 4 wheels) with primary and secondary suspensions
- Coupled track model: Rail → Pad → Sleeper → Ballast → Sub-Ballast → Bridge
- Newmark-β time integration (average acceleration or damped variant)
- Rayleigh damping for beam and rail
- Track irregularity profiles: smooth, PSD-based (FRA/German/SNCF), cosine bump
- Batch processing for HSLM-A1–A10 sweep (EN 1991-2) with multiprocessing
- Output: displacement, bending moment, shear force, acceleration (time-history + contour)

---

## Requirements

```
Python >= 3.8
numpy >= 1.20
scipy >= 1.7
matplotlib >= 3.4
```

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## Project Structure

```
Python_Version/
│
├── A00_Run.py                  # Entry point — single simulation
├── A00_Run_HSLM_Sweep.py       # Entry point — HSLM batch sweep
├── requirements.txt
├── README.md
│
└── ttb2d/                      # Main package
    ├── __init__.py
    ├── data_classes.py          # Namespace helpers (Beam, Track, Train, Calc, ...)
    ├── train_properties.py      # Vehicle property definitions
    ├── track_properties.py      # Track property definitions
    ├── plotting.py              # Plotting utilities
    │
    ├── B00_calculations.py      # Main orchestrator
    ├── B01_elements_and_coordinates.py
    ├── B02_boundary_conditions.py
    ├── B03_beam_matrices.py
    ├── B07_options_processing.py
    ├── B08_veh_freq.py
    ├── B09_beam_frq.py
    ├── B10_end_time.py
    ├── B11_time_space_discretization.py
    ├── B14_eq_vert_nodal_force.py
    ├── B17_calc_uat.py
    ├── B18_train_veh_eq.py
    ├── B19_generate_profile.py
    ├── B24_beam_damping.py
    ├── B25_wheel_profiles.py
    ├── B31_beam_bm.py
    ├── B33_beam_shear.py
    ├── B43_model_geometry.py
    ├── B47_veh_static_loads.py
    ├── B49_beam_deformation.py
    ├── B50_element_num_of_force.py
    ├── B51_rail_variables.py
    ├── B53_beam_acceleration.py
    ├── B54_model_matrices.py
    ├── B55_model_bc.py
    ├── B56_model_frq.py
    ├── B58_results_beam_sections.py
    ├── B64_coupled_initial_static.py
    ├── B65_dynamic_calc_coupled_faster.py
    └── B66_contact_force.py
```

---

## Quick Start

### Single simulation

```bash
cd Python_Version
python A00_Run.py
```

Expected output:
```
Building model system matrices ...
 DONE
Performing Dynamic Calculations (Coupled System) ...
Calculation time: 2.9s
All calculations finished successfully

--- Results Summary ---
Mid-span displacement (min): -2.0576 mm
Mid-span BM (max):           989771.17 Nm
Mid-span acceleration (max): 0.1807 m/s2
```

### HSLM batch sweep

```bash
python A00_Run_HSLM_Sweep.py
```

Results are saved to `HSLM_Sweep_Results.pkl` and printed as a table.

---

## User Guide

### 1. Define the Bridge

```python
import types

Beam = types.SimpleNamespace()
Beam.Prop = types.SimpleNamespace()
Beam.Prop.E   = 3.5e10     # Young's modulus [N/m²]
Beam.Prop.I   = 0.52       # Second moment of area [m⁴]
Beam.Prop.rho = 16587      # Mass per unit length [kg/m]
Beam.Prop.L   = 20         # Span length [m]

Beam.Damping = types.SimpleNamespace()
Beam.Damping.per = 2.0     # Damping ratio [%]

Beam.BC = types.SimpleNamespace()
Beam.BC.text = 'SP'        # Boundary condition: 'SP' = Simply Supported, 'FF' = Fixed-Fixed

Beam.Mesh = types.SimpleNamespace()
Beam.Mesh.Ele = types.SimpleNamespace()
Beam.Mesh.Ele.num_per_spacing = 2   # FEM elements per sleeper spacing
```

### 2. Define the Track

Use a built-in track property function:

```python
from ttb2d.track_properties import TrackProp_Zhai_NoBallastOnBridge
# or: TrackProp_Zhai_NoBallastOnBridge() TrackProp_Zhai_WithBallastOnBridge

Track = TrackProp_Zhai_NoBallastOnBridge()
Track.Rail.Mesh.Ele.num_per_spacing = 1   # Rail elements per sleeper spacing
```

**Track parameters (Zhai et al.):**
| Component | Parameter | Value |
|---|---|---|
| Rail | E·I | 2.059×10¹¹ × 6.434×10⁻⁵ N·m² |
| Pad | k / c | 65 MN/m / 75 kN·s/m |
| Sleeper | spacing / mass | 0.6 m / 251 kg |
| Ballast | k / c | 137.75 MN/m / 58.8 kN·s/m |
| Sub-Ballast | k / c | 77.5 MN/m / 31.15 kN·s/m |

### 3. Define the Train

**Option A — Manchester Benchmark vehicle:**

```python
from ttb2d.train_properties import TrainProp_Manchester_BenchMark

Train = types.SimpleNamespace()
Train.vel = 100 / 3.6      # [m/s], convert from km/h
Train.Veh = types.SimpleNamespace()
Train.Veh.data = [TrainProp_Manchester_BenchMark()]
Train.Veh.num = 1
```

**Option B — HSLM-A train (EN 1991-2):**

```python
from ttb2d.train_properties import TrainProp_HSLM

Train.Veh.data = TrainProp_HSLM('A1')   # Returns list of N coaches
Train.Veh.num = len(Train.Veh.data)
```

Available trains: `'A1'` through `'A10'`

**Custom vehicle:** populate a `SimpleNamespace` with fields:

| Field | Description | Example |
|---|---|---|
| `Body.m` | Body mass [kg] | 32000 |
| `Body.I` | Body moment of inertia [kg·m²] | 1970000 |
| `Body.L` | Distance between bogie centres [m] | 19.0 |
| `Body.Le` | End lengths [m, m] | [3.0, 3.0] |
| `Bogie.m` | Bogie masses [kg, kg] | [2615, 2615] |
| `Bogie.I` | Bogie inertias [kg·m², kg·m²] | [1476, 1476] |
| `Bogie.L` | Bogie wheel spacing [m, m] | [2.56, 2.56] |
| `Wheels.m` | Wheel masses [kg×4] | [1813, 1813, 1813, 1813] |
| `Susp.Prim.k/c` | Primary suspension k/c [N/m×4, N·s/m×4] | |
| `Susp.Sec.k/c` | Secondary suspension k/c [N/m×2, N·s/m×2] | |

### 4. Set Calculation Options

```python
Calc = types.SimpleNamespace()

Calc.Profile = types.SimpleNamespace()
Calc.Profile.Type = 0           # 0=Smooth, 1=PSD random, 3=Cosine bump
Calc.Profile.minL_Approach = 20 # Minimum approach length [m]

Calc.Options = types.SimpleNamespace()
Calc.Options.redux  = 1    # 1=Redux model (rail only on bridge), 0=Full model
Calc.Options.VBI    = 1    # 1=Full VBI, 0=Moving force only
Calc.Options.calc_model_frq   = 0  # 1=Compute system frequencies
Calc.Options.calc_model_modes = 0  # 1=Compute system mode shapes
```

**Profile types:**

| `Type` | Description |
|---|---|
| `0` | Smooth track (no irregularity) |
| `1` | Stochastic PSD-based irregularity (set `Calc.Profile.PSD_type`) |
| `3` | Cosine bump at specified location |

**PSD types** (used when `Type=1`):  `'FRA_6'`, `'FRA_5'`, `'German_low'`, `'German_high'`, `'SNCF'`

```python
Calc.Profile.Type     = 1
Calc.Profile.PSD_type = 'FRA_6'
Calc.Profile.seed     = 42    # Random seed (0 = random each run)
```

### 5. Run the Simulation

```python
from ttb2d.B00_calculations import B00_Calculations

Calc, Train, Track, Beam, Model, Sol = B00_Calculations(Calc, Train, Track, Beam)
```

### 6. Access Results

All results are stored in the `Sol` object:

```python
# Mid-span values
Sol.Beam.U.min05        # Min mid-span displacement [m]
Sol.Beam.BM.max05       # Max mid-span bending moment [N·m]
Sol.Beam.Acc.max05      # Max mid-span acceleration [m/s²]
Sol.Beam.Shear.max      # Max shear force [N]

# Full time-history (nodes × time steps)
Sol.Beam.U.xt           # Displacement [m]  shape: (N_nodes, N_t)
Sol.Beam.BM.xt          # Bending moment [N·m]
Sol.Beam.Shear.xt       # Shear force [N]
Sol.Beam.Acc.xt         # Acceleration [m/s²]

# Contact information
Sol.contactLost         # True if contact loss detected

# Vehicle response
Sol.Veh[i].U            # Vehicle DOF displacements (6 × N_t)
```

### 7. Plotting

```python
import matplotlib.pyplot as plt
from ttb2d.plotting import C01_MidSpanTimeHistory, C02_TimeHistoryPlot, C04_HSLM_Summary

# Mid-span time history
fig = C01_MidSpanTimeHistory(Sol, Beam, Calc)
plt.show()

# Contour plots (displacement, BM, shear, acceleration)
fig = C02_TimeHistoryPlot(Sol, Beam, Calc, Model, Train)
plt.show()
```

### 8. HSLM Sweep Configuration

Edit `A00_Run_HSLM_Sweep.py` to customise:

```python
train_names    = [f'A{i}' for i in range(1, 11)]   # A1–A10
velocities_kmh = np.arange(100, 420 + 1, 20)       # 100–420 km/h, step 20
n_workers      = max(1, cpu_count() - 1)            # Parallel workers
```

Load saved results:

```python
import pickle
with open('HSLM_Sweep_Results.pkl', 'rb') as f:
    results = pickle.load(f)

from ttb2d.plotting import C04_HSLM_Summary
import matplotlib.pyplot as plt
fig = C04_HSLM_Summary(results)
plt.show()
```

---

## Newmark-β Solver Settings

```python
Calc.Solver = types.SimpleNamespace()
Calc.Solver.NewMark_damp = 0    # 0 = average acceleration (β=0.25, δ=0.5)
                                # 1 = damped (β=0.3025, δ=0.6)
```

Time step is computed automatically from beam/vehicle frequencies and element size.

---

## Physical Model

```
  Body (m_b, I_b)
   |          |
  Sec        Sec
  Susp       Susp (k_s, c_s)
   |          |
 Bogie1     Bogie2 (m_t, I_t)
  | |         | |
 Prim       Prim (k_p, c_p)
  | |         | |
 W1 W2      W3 W4  (m_w) ← wheels
  ||||||||||||||||||||     ← Rail (Euler-Bernoulli beam)
  ===Pad===  ===Pad===     (k_pad, c_pad)
  [Sleeper]  [Sleeper]     (m_sl, spacing)
  =Ballast=  =Ballast=     (k_b, c_b)
 =SubBallast=              (k_sb, c_sb)
══════════════════════════  ← Bridge beam (Euler-Bernoulli)
```

---

## Limitations

- 2D model only (vertical dynamics, no lateral or longitudinal)
- Single railway track, single-span bridge
- Constant train velocity (acceleration not yet fully validated for all cases)
- Plotting functions are basic; for advanced post-processing, use `Sol` arrays directly

---

## Citation

---

## License

GNU General Public License v3.0 — see [LICENSE](../LICENSE)
