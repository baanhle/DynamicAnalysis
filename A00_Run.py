"""
A00_Run - Main entry point for a single TTB-2D simulation.
Equivalent to A00_Run.m.

Usage:
    python A00_Run.py
"""
import types
import numpy as np
import sys
import os

# Add parent dir to path so ttb2d package is importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ttb2d.B00_calculations import B00_Calculations
from ttb2d.train_properties import TrainProp_Manchester_BenchMark
from ttb2d.track_properties import TrackProp_Zhai_WithBallastOnBridge


def main():
    # =====================================================================
    # ---- User Input ----
    # =====================================================================

    # ---- Train ----
    Train = types.SimpleNamespace()
    Train.vel = 100 / 3.6  # Velocity [m/s]

    # Vehicle definition (single vehicle)
    veh_list = [TrainProp_Manchester_BenchMark()]
    Train.Veh = types.SimpleNamespace()
    Train.Veh.data = veh_list
    Train.Veh.num = len(veh_list)

    # ---- Track ----
    Track = TrackProp_Zhai_WithBallastOnBridge()
    Track.Rail.Mesh.Ele.num_per_spacing = 1

    # ---- Bridge ----
    Beam = types.SimpleNamespace()
    Beam.Prop = types.SimpleNamespace()
    Beam.Prop.E = 3.5e10        # Young's modulus [N/m2]
    Beam.Prop.I = 0.52          # Second moment of area [m4]
    Beam.Prop.rho = 16587       # Mass per unit length [kg/m] (includes ballast etc.)
    Beam.Prop.L = 20            # Beam length [m]
    Beam.Damping = types.SimpleNamespace()
    Beam.Damping.per = 2.0      # Damping [%]
    Beam.BC = types.SimpleNamespace()
    Beam.BC.text = 'SP'         # Simply Supported

    Beam.Mesh = types.SimpleNamespace()
    Beam.Mesh.Ele = types.SimpleNamespace()
    Beam.Mesh.Ele.num_per_spacing = 2

    # ---- Calculation options ----
    Calc = types.SimpleNamespace()
    Calc.Profile = types.SimpleNamespace()
    Calc.Profile.Type = 0       # Smooth (no irregularity)
    Calc.Profile.minL_Approach = 20  # Minimum approach distance [m]

    Calc.Options = types.SimpleNamespace()
    Calc.Options.redux = 1      # Redux model
    Calc.Options.VBI = 1        # Vehicle-Bridge Interaction ON
    Calc.Options.calc_model_frq = 0
    Calc.Options.calc_model_modes = 0

    # =====================================================================
    # ---- Run Simulation ----
    # =====================================================================
    Calc, Train, Track, Beam, Model, Sol = B00_Calculations(Calc, Train, Track, Beam)

    # =====================================================================
    # ---- Display Results ----
    # =====================================================================
    print('\n--- Results Summary ---')
    print(f'Mid-span displacement (min): {Sol.Beam.U.min05 * 1000:.4f} mm')
    print(f'Mid-span BM (max):           {Sol.Beam.BM.max05:.2f} Nm')
    print(f'Mid-span acceleration (max): {Sol.Beam.Acc.max05:.4f} m/s2')
    if hasattr(Sol.Beam, 'Shear'):
        print(f'Max shear:                   {Sol.Beam.Shear.max:.2f} N')

    return Calc, Train, Track, Beam, Model, Sol


if __name__ == '__main__':
    main()
