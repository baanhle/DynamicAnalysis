"""
A00_Run_HSLM_Sweep - Batch process for HSLM-A1 to A10 trains over a velocity range.
Equivalent to A00_Run_HSLM_Sweep.m.

Uses multiprocessing to parallelize across trains/velocities.

Usage:
    python A00_Run_HSLM_Sweep.py
"""
import types
import numpy as np
import sys
import os
import time
import pickle
from multiprocessing import Pool, cpu_count
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ttb2d.B00_calculations import B00_Calculations
from ttb2d.train_properties import TrainProp_HSLM, HSLM_PARAMS
from ttb2d.track_properties import TrackProp_Zhai_WithBallastOnBridge
from ttb2d.plotting import C04_HSLM_Summary_Plots


def make_beam():
    Beam = types.SimpleNamespace()
    Beam.Prop = types.SimpleNamespace()
    Beam.Prop.E = 3.5e10
    Beam.Prop.I = 51.3
    Beam.Prop.rho = 69000
    Beam.Prop.L = 50
    Beam.Damping = types.SimpleNamespace()
    Beam.Damping.per = 2.0
    Beam.BC = types.SimpleNamespace()
    Beam.BC.text = 'SP'
    Beam.Mesh = types.SimpleNamespace()
    Beam.Mesh.Ele = types.SimpleNamespace()
    Beam.Mesh.Ele.num_per_spacing = 1
    return Beam


def compute_beam_natural_frequencies_hz(beam, n_modes=3):
    """Compute simply-supported beam natural frequencies in Hz."""
    n = np.arange(1, n_modes + 1, dtype=float)
    omega_n = (n ** 2) * (np.pi ** 2) * np.sqrt(
        (beam.Prop.E * beam.Prop.I) / (beam.Prop.rho * (beam.Prop.L ** 4))
    )
    return omega_n / (2.0 * np.pi)


def make_calc():
    Calc = types.SimpleNamespace()
    Calc.Profile = types.SimpleNamespace()
    Calc.Profile.Type = 0
    Calc.Profile.minL_Approach = 2.4
    Calc.Options = types.SimpleNamespace()
    Calc.Options.redux = 1
    Calc.Options.VBI = 1
    Calc.Options.calc_model_frq = 0
    Calc.Options.calc_model_modes = 0
    Calc.Options.beam_frq_factor = 4.0
    Calc.Options.veh_frq_factor  = 4.0
    Calc.Options.min_Nele        = 1.0
    return Calc


def run_single(args):
    """Worker function for one (train_name, velocity) combination."""
    train_name, vel_kmh, num_coaches = args
    try:
        N, D, d, P = HSLM_PARAMS[train_name]
        if num_coaches is None:
            num_coaches = N

        # Train
        Train = types.SimpleNamespace()
        Train.vel = vel_kmh / 3.6
        veh_list = TrainProp_HSLM(train_name, num_coaches)
        Train.Veh = types.SimpleNamespace()
        Train.Veh.data = veh_list
        Train.Veh.num = len(veh_list)

        # Track
        Track = TrackProp_Zhai_WithBallastOnBridge()
        Track.Rail.Mesh.Ele.num_per_spacing = 1

        # Beam & Calc
        Beam = make_beam()
        Calc = make_calc()

        # Run
        Calc, Train, Track, Beam, Model, Sol = B00_Calculations(Calc, Train, Track, Beam)

        result = {
            'train': train_name,
            'vel_kmh': vel_kmh,
            'disp_min05': Sol.Beam.U.min05,
            'BM_max05': Sol.Beam.BM.max05,
            'Acc_max05': Sol.Beam.Acc.max05,
            'contactLost': Sol.contactLost,
        }
        if hasattr(Sol.Beam, 'Shear'):
            result['Shear_max'] = Sol.Beam.Shear.max
        return result

    except Exception as e:
        return {
            'train': train_name,
            'vel_kmh': vel_kmh,
            'error': str(e),
        }


def main():
    # =====================================================================
    # ---- Configuration ----
    # =====================================================================
    train_names = [f'A{i}' for i in range(1, 11)]  # A1 to A10
    velocities_kmh = np.arange(250, 350 + 1, 10)   # 250 to 350 km/h, step 10
    num_coaches = None  # Use default (N from EN 1991-2)

    # Build job list
    jobs = []
    for tn in train_names:
        for vel in velocities_kmh:
            jobs.append((tn, float(vel), num_coaches))

    total = len(jobs)
    print(f'HSLM Sweep: {len(train_names)} trains x {len(velocities_kmh)} velocities = {total} jobs')

    # =====================================================================
    # ---- Run ----
    # =====================================================================
    t0 = time.time()
    n_workers = max(1, cpu_count() - 1)
    print(f'Using {n_workers} parallel workers')

    results = []
    with Pool(n_workers) as pool:
        for i, res in enumerate(pool.imap_unordered(run_single, jobs)):
            results.append(res)
            if (i + 1) % 10 == 0 or (i + 1) == total:
                print(f'  Completed {i + 1}/{total}')

    elapsed = time.time() - t0
    print(f'\nAll {total} jobs completed in {elapsed:.1f}s')

    # =====================================================================
    # ---- Organize Results ----
    # =====================================================================
    # Save raw results
    with open('HSLM_Sweep_Results.pkl', 'wb') as f:
        pickle.dump(results, f)

    # Print summary table
    print('\n--- Results Summary ---')
    print(f'{"Train":<6} {"Vel(km/h)":<10} {"Disp(mm)":<12} {"BM(Nm)":<12} {"Acc(m/s2)":<12}')
    print('-' * 52)
    for r in sorted(results, key=lambda x: (x.get('train', ''), x.get('vel_kmh', 0))):
        if 'error' in r:
            print(f'{r["train"]:<6} {r["vel_kmh"]:<10.0f} ERROR: {r["error"]}')
        else:
            print(f'{r["train"]:<6} {r["vel_kmh"]:<10.0f} '
                  f'{r["disp_min05"]*1000:<12.4f} '
                  f'{r["BM_max05"]:<12.2f} '
                  f'{r["Acc_max05"]:<12.4f}')

    # Generate C04-style summary plots (displacement, acceleration, worst train, natural frequency)
    beam_for_freq = make_beam()
    fn_hz = compute_beam_natural_frequencies_hz(beam_for_freq, n_modes=3)
    figs = C04_HSLM_Summary_Plots(
        results=results,
        eurocode_acc_limit=3.5,
        beam_natural_freq_hz=fn_hz,
        excitation_lengths_m=np.array([18.0, 27.0]),
    )

    figs['fig_disp'].savefig('HSLM_Sweep_Displacement.png', dpi=180, bbox_inches='tight')
    figs['fig_acc'].savefig('HSLM_Sweep_Acceleration.png', dpi=180, bbox_inches='tight')
    figs['fig_worst'].savefig('HSLM_Sweep_Critical_Train.png', dpi=180, bbox_inches='tight')
    figs['fig_freq'].savefig('HSLM_Sweep_NaturalFrequency_CriticalSpeed.png', dpi=180, bbox_inches='tight')
    plt.close('all')

    print('\nSaved plot files:')
    print('  - HSLM_Sweep_Displacement.png')
    print('  - HSLM_Sweep_Acceleration.png')
    print('  - HSLM_Sweep_Critical_Train.png')
    print('  - HSLM_Sweep_NaturalFrequency_CriticalSpeed.png')

    return results


if __name__ == '__main__':
    main()
