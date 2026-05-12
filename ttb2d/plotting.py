"""
Plotting utilities for TTB-2D Python version.
Provides time-history plots, contour plots, and HSLM summary plots.
"""
import numpy as np
import matplotlib.pyplot as plt
import pickle


def C02_TimeHistoryPlot(Sol, Beam, Calc, Model, Train):
    """Generate time-history and contour plots for beam results."""
    t = Calc.Solver.t
    x = Beam.Mesh.Nodes.acum

    fig, axes = plt.subplots(4, 1, figsize=(12, 16), sharex=False)

    # Displacement contour
    ax = axes[0]
    if hasattr(Sol.Beam, 'U') and hasattr(Sol.Beam.U, 'xt'):
        im = ax.pcolormesh(t, x, Sol.Beam.U.xt * 1000, shading='auto', cmap='RdBu_r')
        plt.colorbar(im, ax=ax, label='Disp [mm]')
    ax.set_ylabel('Position [m]')
    ax.set_title('Beam Vertical Displacement')

    # BM contour
    ax = axes[1]
    if hasattr(Sol.Beam, 'BM') and hasattr(Sol.Beam.BM, 'xt'):
        im = ax.pcolormesh(t, x, Sol.Beam.BM.xt, shading='auto', cmap='RdBu_r')
        plt.colorbar(im, ax=ax, label='BM [Nm]')
    ax.set_ylabel('Position [m]')
    ax.set_title('Beam Bending Moment')

    # Shear contour
    ax = axes[2]
    if hasattr(Sol.Beam, 'Shear') and hasattr(Sol.Beam.Shear, 'xt'):
        im = ax.pcolormesh(t, x, Sol.Beam.Shear.xt, shading='auto', cmap='RdBu_r')
        plt.colorbar(im, ax=ax, label='Shear [N]')
    ax.set_ylabel('Position [m]')
    ax.set_title('Beam Shear Force')

    # Acceleration contour
    ax = axes[3]
    if hasattr(Sol.Beam, 'Acc') and hasattr(Sol.Beam.Acc, 'xt'):
        im = ax.pcolormesh(t, x, Sol.Beam.Acc.xt, shading='auto', cmap='RdBu_r')
        plt.colorbar(im, ax=ax, label='Acc [m/s²]')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Position [m]')
    ax.set_title('Beam Acceleration')

    plt.tight_layout()
    return fig


def C01_MidSpanTimeHistory(Sol, Beam, Calc):
    """Plot mid-span displacement time history."""
    mid = Beam.Mesh.Nodes.Tnum // 2
    t = Calc.Solver.t

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    if hasattr(Sol.Beam, 'U'):
        axes[0].plot(t, Sol.Beam.U.xt[mid, :] * 1000, 'b-')
        if hasattr(Sol.Beam, 'StaticU'):
            axes[0].plot(t, Sol.Beam.StaticU.xt[mid, :] * 1000, 'r--', label='Static')
        axes[0].set_ylabel('Displacement [mm]')
        axes[0].legend()

    if hasattr(Sol.Beam, 'BM'):
        axes[1].plot(t, Sol.Beam.BM.xt[mid, :], 'b-')
        if hasattr(Sol.Beam, 'StaticBM'):
            axes[1].plot(t, Sol.Beam.StaticBM.xt[mid, :], 'r--', label='Static')
        axes[1].set_ylabel('BM [Nm]')

    if hasattr(Sol.Beam, 'Acc'):
        axes[2].plot(t, Sol.Beam.Acc.xt[mid, :], 'b-')
        axes[2].set_ylabel('Acceleration [m/s²]')

    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    return fig


def C04_HSLM_Summary(results, velocities_kmh=None):
    """
    Plot summary of HSLM sweep results.
    results: list of dicts from A00_Run_HSLM_Sweep.
    """
    # Organize by train
    trains = sorted(set(r['train'] for r in results if 'error' not in r))

    fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True)

    for tn in trains:
        data = sorted([r for r in results if r.get('train') == tn and 'error' not in r],
                      key=lambda x: x['vel_kmh'])
        vels = [d['vel_kmh'] for d in data]
        disps = [abs(d['disp_min05']) * 1000 for d in data]
        bms = [d['BM_max05'] for d in data]
        accs = [d['Acc_max05'] for d in data]

        axes[0].plot(vels, disps, '-o', markersize=3, label=tn)
        axes[1].plot(vels, bms, '-o', markersize=3, label=tn)
        axes[2].plot(vels, accs, '-o', markersize=3, label=tn)

    axes[0].set_ylabel('|Mid-span Disp| [mm]')
    axes[0].legend(ncol=5, fontsize=8)
    axes[0].grid(True)

    axes[1].set_ylabel('Mid-span BM [Nm]')
    axes[1].grid(True)

    axes[2].set_ylabel('Mid-span Acc [m/s²]')
    axes[2].set_xlabel('Velocity [km/h]')
    axes[2].grid(True)

    plt.suptitle('HSLM-A Sweep Results', fontsize=14)
    plt.tight_layout()
    return fig


def load_sweep_results(file_path='HSLM_Sweep_Results.pkl'):
    """Load HSLM sweep results from pickle file."""
    with open(file_path, 'rb') as f:
        results = pickle.load(f)
    return results


def _organize_sweep_results(results):
    """Organize list-of-dict sweep output into arrays by train and velocity."""
    clean = [r for r in results if 'error' not in r]
    if len(clean) == 0:
        raise ValueError('No valid sweep results found (all runs contain errors).')

    trains = sorted(set(r['train'] for r in clean), key=lambda x: (len(x), x))
    velocities = sorted(set(float(r['vel_kmh']) for r in clean))

    n_t = len(trains)
    n_v = len(velocities)

    disp_mm = np.full((n_t, n_v), np.nan)
    acc = np.full((n_t, n_v), np.nan)
    bm = np.full((n_t, n_v), np.nan)

    train_to_i = {t: i for i, t in enumerate(trains)}
    vel_to_i = {v: i for i, v in enumerate(velocities)}

    for r in clean:
        i = train_to_i[r['train']]
        j = vel_to_i[float(r['vel_kmh'])]
        disp_mm[i, j] = abs(float(r['disp_min05'])) * 1000.0
        acc[i, j] = float(r['Acc_max05'])
        bm[i, j] = float(r['BM_max05'])

    return {
        'trains': trains,
        'velocities': np.array(velocities, dtype=float),
        'disp_mm': disp_mm,
        'acc': acc,
        'bm': bm,
    }


def _compute_beam_natural_frequencies_hz(beam_E, beam_I, beam_rho, beam_L, n_modes=3):
    """Simply-supported Euler-Bernoulli beam natural frequencies in Hz."""
    n = np.arange(1, n_modes + 1, dtype=float)
    omega_n = (n ** 2) * (np.pi ** 2) * np.sqrt((beam_E * beam_I) / (beam_rho * (beam_L ** 4)))
    return omega_n / (2.0 * np.pi)


def C04_HSLM_Summary_Plots(
    results=None,
    results_file='HSLM_Sweep_Results.pkl',
    eurocode_acc_limit=3.5,
    beam_natural_freq_hz=None,
    excitation_lengths_m=None,
):
    """
    MATLAB-like C04 summary plots for HSLM sweep.

    Returns a dict of matplotlib figures:
      - fig_disp: max displacement envelope
      - fig_acc: max acceleration envelope (+ Eurocode limit)
      - fig_worst: worst displacement per train
      - fig_freq: natural frequency / critical speed reference
    """
    if results is None:
        results = load_sweep_results(results_file)

    data = _organize_sweep_results(results)
    v = data['velocities']
    trains = data['trains']
    disp_mm = data['disp_mm']
    acc = data['acc']

    legends = [f'HSLM-{t}' for t in trains]

    # ---- Plot 1: Maximum Displacement ----
    fig_disp, ax = plt.subplots(figsize=(9, 5), num='HSLM-A Sweep: Bridge Displacement')
    for i in range(len(trains)):
        ax.plot(v, disp_mm[i, :], '-o', linewidth=1.5, markersize=4)
    ax.grid(True)
    ax.set_xlabel('Velocity [km/h]')
    ax.set_ylabel('Maximum Displacement [mm]')
    ax.set_title('Bridge Center Maximum Displacement Envelope')
    ax.legend(legends, loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=9)
    fig_disp.tight_layout()

    # ---- Plot 2: Maximum Acceleration ----
    fig_acc, ax = plt.subplots(figsize=(9, 5), num='HSLM-A Sweep: Bridge Acceleration')
    for i in range(len(trains)):
        ax.plot(v, acc[i, :], '-s', linewidth=1.5, markersize=4)
    ax.grid(True)
    ax.set_xlabel('Velocity [km/h]')
    ax.set_ylabel('Maximum Acceleration [m/s^2]')
    ax.set_title('Bridge Maximum Vertical Acceleration Envelope')
    ax.axhline(eurocode_acc_limit, color='r', linestyle='--', linewidth=2)
    ax.text(v.min() + 5, eurocode_acc_limit + 0.2, f'Eurocode limit ({eurocode_acc_limit:.1f} m/s^2)',
            color='r', fontweight='bold')
    ax.legend(legends, loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=9)
    fig_acc.tight_layout()

    # ---- Plot 3: Worst-case displacement per train ----
    worst_disp = np.nanmax(disp_mm, axis=1)
    fig_worst, ax = plt.subplots(figsize=(8, 4.8), num='HSLM-A Summary: Critical Train')
    ax.bar(np.arange(len(trains)), worst_disp, color=(0.7, 0.0, 0.0))
    ax.set_xticks(np.arange(len(trains)))
    ax.set_xticklabels(legends, rotation=45, ha='right')
    ax.set_ylabel('Max Absolute Displacement [mm]')
    ax.set_title('Critical HSLM-A Train Comparison')
    ax.grid(True, axis='y')
    fig_worst.tight_layout()

    # ---- Plot 4: Natural frequencies and equivalent critical speeds ----
    if beam_natural_freq_hz is None:
        # Defaults from current benchmark beam settings (same as A00 scripts)
        beam_natural_freq_hz = _compute_beam_natural_frequencies_hz(
            beam_E=3.5e10,
            beam_I=0.52,
            beam_rho=16587,
            beam_L=20.0,
            n_modes=3,
        )
    beam_natural_freq_hz = np.atleast_1d(beam_natural_freq_hz).astype(float)

    if excitation_lengths_m is None:
        # HSLM coach length D range from EN 1991-2 (A1...A10): 18 to 27 m
        excitation_lengths_m = np.array([18.0, 27.0])
    excitation_lengths_m = np.sort(np.atleast_1d(excitation_lengths_m).astype(float))

    fig_freq, axes = plt.subplots(2, 1, figsize=(9, 7), num='Natural Frequency and Critical Speed')

    # 4a) beam natural frequencies
    mode_ids = np.arange(1, len(beam_natural_freq_hz) + 1)
    axes[0].bar(mode_ids, beam_natural_freq_hz, color='#2f6db3')
    axes[0].set_xticks(mode_ids)
    axes[0].set_xlabel('Mode number')
    axes[0].set_ylabel('Natural Frequency [Hz]')
    axes[0].set_title('Bridge Natural Frequencies')
    axes[0].grid(True, axis='y')

    # 4b) equivalent critical-speed bands: v = 3.6 * f_n * L_exc
    for i, fn in enumerate(beam_natural_freq_hz):
        v_low = 3.6 * fn * excitation_lengths_m[0]
        v_high = 3.6 * fn * excitation_lengths_m[-1]
        axes[1].hlines(i + 1, v_low, v_high, colors='tab:orange', linewidth=6)
        axes[1].plot([v_low, v_high], [i + 1, i + 1], 'ko', markersize=4)
    axes[1].set_xlabel('Equivalent Critical Speed [km/h]')
    axes[1].set_ylabel('Mode index')
    axes[1].set_yticks(mode_ids)
    axes[1].set_yticklabels([f'Mode {i}' for i in mode_ids])
    axes[1].set_title(
        f'Critical Speed Bands from Excitation Length {excitation_lengths_m[0]:.0f}-{excitation_lengths_m[-1]:.0f} m'
    )
    axes[1].grid(True)
    fig_freq.tight_layout()

    return {
        'fig_disp': fig_disp,
        'fig_acc': fig_acc,
        'fig_worst': fig_worst,
        'fig_freq': fig_freq,
        'data': data,
    }


def C03_TTB_2D_Plots(Calc, Train, Track, Beam, Model, Sol):
    """
    MATLAB-like high-level plotting entry point (partial C03 equivalent).
    Uses existing Python plotting helpers and returns generated figures.
    """
    figs = {}
    figs['midspan'] = C01_MidSpanTimeHistory(Sol, Beam, Calc)
    figs['contours'] = C02_TimeHistoryPlot(Sol, Beam, Calc, Model, Train)
    return figs
