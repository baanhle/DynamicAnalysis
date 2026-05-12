"""
core_worker.py - Computation engine for the HSLM Web Application.

Provides:
  - check_free_vibration(params) -> dict
  - run_single_job(args)         -> dict  (top-level; pickleable for ProcessPoolExecutor)
  - run_dynamic_sweep(params, update_cb) -> dict
"""
from __future__ import annotations

import io
import base64
import types
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Non-interactive backend – safe for server use
import matplotlib.pyplot as plt

import sys
import os

# Make ttb2d importable regardless of where the worker runs
_BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BASE_DIR not in sys.path:
    sys.path.insert(0, _BASE_DIR)

from ttb2d.B00_calculations import B00_Calculations
from ttb2d.train_properties import TrainProp_HSLM, HSLM_PARAMS
from ttb2d.track_properties import (
    TrackProp_Zhai_WithBallastOnBridge,
    TrackProp_Zhai_NoBallastOnBridge,
)
from ttb2d.plotting import C04_HSLM_Summary_Plots


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _fig_to_b64(fig: plt.Figure) -> str:
    """Render a matplotlib figure to a PNG base64 string."""
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=150, bbox_inches="tight")
    buf.seek(0)
    data = base64.b64encode(buf.read()).decode("utf-8")
    plt.close(fig)
    return data


def _make_beam(p: dict) -> types.SimpleNamespace:
    Beam = types.SimpleNamespace()
    Beam.Prop = types.SimpleNamespace()
    Beam.Prop.E = float(p["E"])
    Beam.Prop.I = float(p.get("Ixx", p["I"]))
    Beam.Prop.rho = float(p["rho"])
    Beam.Prop.L = float(p["L"])
    Beam.Damping = types.SimpleNamespace()
    Beam.Damping.per = float(p["damping_pct"])
    Beam.BC = types.SimpleNamespace()
    Beam.BC.text = "SP"
    Beam.Mesh = types.SimpleNamespace()
    Beam.Mesh.Ele = types.SimpleNamespace()
    Beam.Mesh.Ele.num_per_spacing = int(p.get("ele_per_spacing", 2))
    return Beam


def _make_track(track_type: str) -> types.SimpleNamespace:
    if track_type == "no_ballast":
        return TrackProp_Zhai_NoBallastOnBridge()
    return TrackProp_Zhai_WithBallastOnBridge()


def _make_calc(p: dict = None) -> types.SimpleNamespace:
    Calc = types.SimpleNamespace()
    Calc.Profile = types.SimpleNamespace()
    Calc.Profile.Type = 0
    Calc.Profile.minL_Approach = 20
    
    if p:
        Calc.Profile.Type = int(p.get("type", 0))
        Calc.Profile.PSD_type = p.get("psd_type", "FRA_6")
        Calc.Profile.seed = int(p.get("seed", 42))
        Calc.Profile.amp = float(p.get("amp", 0.002))
        Calc.Profile.length = float(p.get("length", 1.0))

    Calc.Options = types.SimpleNamespace()
    Calc.Options.redux = 1
    Calc.Options.VBI = 1
    Calc.Options.calc_model_frq = 0
    Calc.Options.calc_model_modes = 0
    return Calc


def _beam_natural_frequencies_hz(p: dict, n_modes: int = 3) -> np.ndarray:
    """Simply-supported Euler-Bernoulli beam natural frequencies [Hz]."""
    E = float(p["E"])
    I = float(p["I"])
    rho = float(p["rho"])
    L = float(p["L"])
    n = np.arange(1, n_modes + 1, dtype=float)
    omega_n = (n ** 2) * (np.pi ** 2) * np.sqrt((E * I) / (rho * L ** 4))
    return omega_n / (2.0 * np.pi)


def _principal_inertias(i_xx: float, i_yy: float, i_xy: float) -> tuple[float, float]:
    """Return principal inertias (major, minor) from section inertia tensor."""
    i_avg = 0.5 * (i_xx + i_yy)
    r = np.sqrt((0.5 * (i_xx - i_yy)) ** 2 + i_xy ** 2)
    i1 = i_avg + r
    i2 = i_avg - r
    eps = 1e-9
    return max(i1, eps), max(i2, eps)


def _beam_bending_freq_hz(e: float, i: float, rho: float, L: float, n_modes: int = 3) -> np.ndarray:
    n = np.arange(1, n_modes + 1, dtype=float)
    omega_n = (n ** 2) * (np.pi ** 2) * np.sqrt((e * i) / (rho * L ** 4))
    return omega_n / (2.0 * np.pi)


def _beam_torsion_freq_hz(g: float, j: float, i_theta: float, L: float, n_modes: int = 3) -> np.ndarray:
    """Saint-Venant torsion frequencies for simply-supported uniform beam."""
    n = np.arange(1, n_modes + 1, dtype=float)
    omega_n = (n * np.pi / L) * np.sqrt((g * j) / i_theta)
    return omega_n / (2.0 * np.pi)


def _mode_shape_plot_b64(freqs_hz: np.ndarray, title: str) -> str:
    """Plot first n modal shapes as sin(n*pi*x/L), annotated with frequencies."""
    fig, ax = plt.subplots(figsize=(8.2, 4.6))
    x = np.linspace(0.0, 1.0, 240)
    n_modes = len(freqs_hz)
    for i in range(n_modes):
        n = i + 1
        phi = np.sin(n * np.pi * x)
        ax.plot(x, phi, linewidth=1.8, label=f"Mode {n}: {freqs_hz[i]:.3f} Hz")

    ax.axhline(0.0, color="#666", linewidth=1.0)
    ax.set_xlabel("x / L")
    ax.set_ylabel("Normalized mode shape")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    return _fig_to_b64(fig)


def _single_mode_shape_plot_b64(display_idx: int, order_n: int, freq_hz: float, mode_type: str, L: float) -> str:
    """Plot one mode shape in non-normalized form (raw analytical shape)."""
    fig, ax = plt.subplots(figsize=(4.6, 2.0))
    x = np.linspace(0.0, L, 240)
    # Math uses the harmonic order (order_n = 1, 2, or 3)
    phi = np.sin(order_n * np.pi * x / L)
    
    ax.plot(x, phi, color="#4c51bf", linewidth=2.5)
    ax.axhline(0.0, color="#666", linewidth=1.0, alpha=0.5)
    ax.set_xlim(0.0, L)
    
    # Title uses the global display index (1 to 9) to match the table
    ax.set_title(f"Mode {display_idx}: {mode_type.capitalize()} (n={order_n}) - {freq_hz:.3f} Hz", fontsize=10, fontweight='bold')
    ax.set_xlabel("x (m)", fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.tick_params(labelsize=8)
    fig.tight_layout()
    return _fig_to_b64(fig)


# ---------------------------------------------------------------------------
# API: Free-vibration check
# ---------------------------------------------------------------------------

def check_free_vibration(bridge_p: dict, n_modes: int = 3) -> dict:
    """
    Compute natural frequencies and apply EN 1991-2 simplified check.
    Returns a dict ready to be serialised as VibrationCheckResponse.
    """
    L = float(bridge_p["L"])
    E = float(bridge_p["E"])
    rho = float(bridge_p["rho"])
    Ixx = float(bridge_p.get("Ixx", bridge_p.get("I", 51.3)))
    Iyy = float(bridge_p.get("Iyy", max(Ixx * 0.4, 1e-6)))
    Ixy = float(bridge_p.get("Ixy", 0.0))
    G = float(bridge_p.get("G", E / (2.0 * (1.0 + 0.3))))
    J = float(bridge_p.get("J", 5.0))
    I_theta = float(bridge_p.get("I_theta", 8.0e4))

    i_major, i_minor = _principal_inertias(Ixx, Iyy, Ixy)
    vertical_freqs = _beam_bending_freq_hz(E, i_major, rho, L, n_modes)
    lateral_freqs = _beam_bending_freq_hz(E, i_minor, rho, L, n_modes)
    torsion_freqs = _beam_torsion_freq_hz(G, J, I_theta, L, n_modes)

    f1 = float(min(vertical_freqs[0], lateral_freqs[0], torsion_freqs[0]))

    # EN 1991-2 Table 6.3 – simplified rule:
    # Dynamic analysis required if f1 ≤ f_max(L) where
    # f_max = 2.0 Hz  (often used for spans covered by HSLM)
    # A stricter engineering check uses the Eurocode resonance condition:
    # resonance expected when V_res = f1 * D_coach – within the design speed range.
    # We flag a warning when f1 < 30 Hz (upper practical limit) AND
    # at least one HSLM resonance speed falls in [160, 420] km/h.
    warn = False
    for D in [18, 19, 20, 21, 22, 23, 24, 25, 26, 27]:  # coach lengths A1-A10
        v_res_kmh = f1 * D * 3.6  # resonance speed km/h
        if 160 <= v_res_kmh <= 420:
            warn = True
            break

    if warn:
        verdict = "warning"
        verdict_text = (
            "Kết cấu yêu cầu phân tích động lực học! "
            f"Tốc độ cộng hưởng nằm trong dải thiết kế (160-420 km/h). "
            f"f₁ = {f1:.3f} Hz"
        )
    else:
        verdict = "ok"
        verdict_text = (
            "Kết cấu thỏa mãn an toàn cộng hưởng. Không bắt buộc phân tích động. "
            f"f₁ = {f1:.3f} Hz"
        )

    vertical_modes = [{"mode": i + 1, "freq_hz": float(vertical_freqs[i])} for i in range(n_modes)]
    lateral_modes = [{"mode": i + 1, "freq_hz": float(lateral_freqs[i])} for i in range(n_modes)]
    torsion_modes = [{"mode": i + 1, "freq_hz": float(torsion_freqs[i])} for i in range(n_modes)]

    mode_rows = []
    for i in range(n_modes):
        order_n = i + 1
        mode_rows.append({"mode_type": "lateral",  "order_n": order_n, "freq_hz": float(lateral_freqs[i])})
        mode_rows.append({"mode_type": "vertical", "order_n": order_n, "freq_hz": float(vertical_freqs[i])})
        mode_rows.append({"mode_type": "torsion",  "order_n": order_n, "freq_hz": float(torsion_freqs[i])})

    # Sort all 9 modes by frequency first
    mode_rows.sort(key=lambda m: m["freq_hz"])
    
    # Second pass: assign global mode index and generate plots
    for idx, item in enumerate(mode_rows, start=1):
        item["mode"] = idx
        item["mode_idx"] = idx
        # Generate plot with the global idx for the title and order_n for the math
        item["img_mode_shape"] = _single_mode_shape_plot_b64(
            display_idx=idx,
            order_n=item["order_n"],
            freq_hz=item["freq_hz"],
            mode_type=item["mode_type"],
            L=L
        )

    return {
        "vertical_modes": vertical_modes,
        "lateral_modes": lateral_modes,
        "torsion_modes": torsion_modes,
        "mode_rows": mode_rows,
        # Backward-compatible aliases
        "modes": vertical_modes,
        "f1_hz": float(vertical_freqs[0]),
        "governing_f1_hz": f1,
        "verdict": verdict,
        "verdict_text": verdict_text,
        "img_vertical_modes": _mode_shape_plot_b64(vertical_freqs, "Vertical Bending Modes"),
        "img_lateral_modes": _mode_shape_plot_b64(lateral_freqs, "Lateral Bending Modes"),
        "img_torsion_modes": _mode_shape_plot_b64(torsion_freqs, "Torsional Modes"),
    }


# ---------------------------------------------------------------------------
# API: Dynamic sweep  (top-level worker – must be pickleable on Windows)
# ---------------------------------------------------------------------------

def _run_single_job(args: tuple) -> dict:
    """
    Top-level worker function for ProcessPoolExecutor.
    args = (train_name, vel_kmh, num_coaches, bridge_p, track_type)
    """
    train_name, vel_kmh, num_coaches, bridge_p, track_type = args
    try:
        N, D, d, P = HSLM_PARAMS[train_name]
        if num_coaches is None:
            num_coaches = N

        Train = types.SimpleNamespace()
        Train.vel = vel_kmh / 3.6
        veh_list = TrainProp_HSLM(train_name, num_coaches)
        Train.Veh = types.SimpleNamespace()
        Train.Veh.data = veh_list
        Train.Veh.num = len(veh_list)

        Track = _make_track(track_type)
        Track.Rail.Mesh.Ele.num_per_spacing = 1

        Beam = _make_beam(bridge_p)
        Calc = _make_calc(bridge_p.get("profile"))

        Calc, Train, Track, Beam, Model, Sol = B00_Calculations(Calc, Train, Track, Beam)

        result = {
            "train": train_name,
            "vel_kmh": vel_kmh,
            "disp_min05": float(Sol.Beam.U.min05),
            "BM_max05": float(Sol.Beam.BM.max05),
            "Acc_max05": float(Sol.Beam.Acc.max05),
            "contactLost": bool(Sol.contactLost),
        }
        if hasattr(Sol.Beam, "Shear"):
            result["Shear_max"] = float(Sol.Beam.Shear.max)
        return result

    except Exception as exc:
        return {"train": train_name, "vel_kmh": vel_kmh, "error": str(exc)}


def run_dynamic_sweep(params: dict, update_cb=None) -> dict:
    """
    Run the full HSLM sweep.

    params keys: bridge_p, track_type, train_names, num_coaches,
                 v_min_kmh, v_max_kmh, v_step_kmh
    update_cb(progress: float, text: str) – called after each finished job.

    Returns dict with keys: results, img_disp, img_acc, img_worst, img_freq
    """
    bridge_p = params["bridge_p"]
    track_type = params.get("track_type", "with_ballast")
    train_names = params.get("train_names", [f"A{i}" for i in range(1, 11)])
    num_coaches = params.get("num_coaches", None)
    v_min = float(params.get("v_min_kmh", 250))
    v_max = float(params.get("v_max_kmh", 350))
    v_step = float(params.get("v_step_kmh", 10))

    velocities = np.arange(v_min, v_max + v_step * 0.5, v_step).tolist()

    jobs = [
        (tn, vel, num_coaches, bridge_p, track_type)
        for tn in train_names
        for vel in velocities
    ]
    total = len(jobs)

    results = []
    completed = 0
    max_workers = int(params.get("max_workers", max(1, (os.cpu_count() or 2) - 1)))

    if update_cb is not None:
        update_cb(0.01, f"Đang chuẩn bị {total} tổ hợp tính toán (parallel {max_workers} workers)...")

    from concurrent.futures import ProcessPoolExecutor, as_completed

    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = {executor.submit(_run_single_job, job): job for job in jobs}
        for fut in as_completed(futures):
            res = fut.result()
            results.append(res)
            completed += 1
            if update_cb is not None:
                tn = res.get("train", "?")
                vel = res.get("vel_kmh", 0)
                progress = max(0.01, min(0.99, completed / total))
                update_cb(progress, f"Hoàn thành {completed}/{total} – Tàu {tn} @ {vel:.0f} km/h")

    valid_results = [r for r in results if "error" not in r]
    if len(valid_results) == 0:
        sample = "; ".join(
            [f"{r.get('train', '?')}@{r.get('vel_kmh', '?')}: {r.get('error', 'unknown')}" for r in results[:3]]
        )
        raise RuntimeError(f"All sweep jobs failed. Sample errors: {sample}")

    # Generate plots
    freqs_hz = _beam_natural_frequencies_hz(bridge_p, n_modes=3)
    figs = C04_HSLM_Summary_Plots(
        results=results,
        eurocode_acc_limit=3.5,
        beam_natural_freq_hz=freqs_hz,
        excitation_lengths_m=np.array([18.0, 27.0]),
    )

    return {
        "results": results,
        "img_disp": _fig_to_b64(figs["fig_disp"]),
        "img_acc": _fig_to_b64(figs["fig_acc"]),
        "img_worst": _fig_to_b64(figs["fig_worst"]),
        "img_freq": _fig_to_b64(figs["fig_freq"]),
    }
