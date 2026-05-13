"""
core_worker.py - Computation engine for the HSLM Web Application.
"""
from __future__ import annotations

import io
import base64
import types
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import sys
import os

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

def _fig_to_b64(fig: plt.Figure, dpi: int = 120) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight")
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
    Beam.Mesh.Ele.num_per_spacing = int(p.get("ele_per_spacing", 1))
    return Beam


def _make_track(track_type: str) -> types.SimpleNamespace:
    if track_type == "no_ballast":
        return TrackProp_Zhai_NoBallastOnBridge()
    return TrackProp_Zhai_WithBallastOnBridge()


def _make_calc(p: dict = None, fast_mode: bool = False) -> types.SimpleNamespace:
    Calc = types.SimpleNamespace()
    Calc.Profile = types.SimpleNamespace()
    Calc.Profile.Type = 0
    # Reduce approach length to bare minimum to drastically shrink rail/sleeper/ballast global DOFs
    Calc.Profile.minL_Approach = 2.4

    if p:
        Calc.Profile.Type = int(p.get("type", 0))
        Calc.Profile.PSD_type = p.get("psd_type", "FRA_6")
        Calc.Profile.seed = int(p.get("seed", 42))
        Calc.Profile.amp = float(p.get("amp", 0.002))
        Calc.Profile.length = float(p.get("length", 1.0))
        Calc.Profile.minL_After = 0.0
    else:
        Calc.Profile.minL_After = 0.0

    Calc.Options = types.SimpleNamespace()
    Calc.Options.redux = 1
    Calc.Options.VBI = 0 if fast_mode else 1
    Calc.Options.calc_model_frq = 0
    Calc.Options.calc_model_modes = 0
    
    # Two presets: balanced default and aggressive fast mode for constrained servers.
    if fast_mode:
        Calc.Options.beam_frq_factor = 2.5
        Calc.Options.veh_frq_factor  = 2.5
        Calc.Options.min_Nele        = 1.5
        Calc.Options.tail_duration   = 5.0
        Calc.Options.tail_dt         = 0.012
    else:
        Calc.Options.beam_frq_factor = 3.0
        Calc.Options.veh_frq_factor  = 3.0
        Calc.Options.min_Nele        = 2.0
        Calc.Options.tail_duration   = 4.0
        Calc.Options.tail_dt         = 0.01
    return Calc


def _beam_natural_frequencies_hz(p: dict, n_modes: int = 3) -> np.ndarray:
    E = float(p["E"])
    I = float(p["I"])
    rho = float(p["rho"])
    L = float(p["L"])
    n = np.arange(1, n_modes + 1, dtype=float)
    omega_n = (n ** 2) * (np.pi ** 2) * np.sqrt((E * I) / (rho * L ** 4))
    return omega_n / (2.0 * np.pi)


def _principal_inertias(i_xx: float, i_yy: float, i_xy: float) -> tuple[float, float]:
    i_avg = 0.5 * (i_xx + i_yy)
    r = np.sqrt((0.5 * (i_xx - i_yy)) ** 2 + i_xy ** 2)
    i1 = i_avg + r
    i2 = i_avg - r
    eps = 1e-9
    return max(i1, eps), max(i2, eps)


def _beam_bending_freq_hz(e, i, rho, L, n_modes=3):
    n = np.arange(1, n_modes + 1, dtype=float)
    omega_n = (n ** 2) * (np.pi ** 2) * np.sqrt((e * i) / (rho * L ** 4))
    return omega_n / (2.0 * np.pi)


def _beam_torsion_freq_hz(g, j, i_theta, L, n_modes=3):
    n = np.arange(1, n_modes + 1, dtype=float)
    omega_n = (n * np.pi / L) * np.sqrt((g * j) / i_theta)
    return omega_n / (2.0 * np.pi)


def _mode_shape_plot_b64(freqs_hz, title):
    fig, ax = plt.subplots(figsize=(8.2, 4.6))
    x = np.linspace(0.0, 1.0, 240)
    for i in range(len(freqs_hz)):
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


def _single_mode_shape_plot_b64(display_idx, order_n, freq_hz, mode_type, L):
    fig, ax = plt.subplots(figsize=(4.6, 2.0))
    x = np.linspace(0.0, L, 240)
    phi = np.sin(order_n * np.pi * x / L)
    ax.plot(x, phi, color="#4c51bf", linewidth=2.5)
    ax.axhline(0.0, color="#666", linewidth=1.0, alpha=0.5)
    ax.set_xlim(0.0, L)
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

    warn = False
    for D in [18, 19, 20, 21, 22, 23, 24, 25, 26, 27]:
        v_res_kmh = f1 * D * 3.6
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
    lateral_modes  = [{"mode": i + 1, "freq_hz": float(lateral_freqs[i])}  for i in range(n_modes)]
    torsion_modes  = [{"mode": i + 1, "freq_hz": float(torsion_freqs[i])}  for i in range(n_modes)]

    mode_rows = []
    for i in range(n_modes):
        order_n = i + 1
        mode_rows.append({"mode_type": "lateral",  "order_n": order_n, "freq_hz": float(lateral_freqs[i])})
        mode_rows.append({"mode_type": "vertical", "order_n": order_n, "freq_hz": float(vertical_freqs[i])})
        mode_rows.append({"mode_type": "torsion",  "order_n": order_n, "freq_hz": float(torsion_freqs[i])})

    mode_rows.sort(key=lambda m: m["freq_hz"])
    for idx, item in enumerate(mode_rows, start=1):
        item["mode"] = idx
        item["mode_idx"] = idx
        item["img_mode_shape"] = _single_mode_shape_plot_b64(
            display_idx=idx, order_n=item["order_n"],
            freq_hz=item["freq_hz"], mode_type=item["mode_type"], L=L
        )

    return {
        "vertical_modes": vertical_modes,
        "lateral_modes": lateral_modes,
        "torsion_modes": torsion_modes,
        "mode_rows": mode_rows,
        "modes": vertical_modes,
        "f1_hz": float(vertical_freqs[0]),
        "governing_f1_hz": f1,
        "verdict": verdict,
        "verdict_text": verdict_text,
        "img_vertical_modes": _mode_shape_plot_b64(vertical_freqs, "Vertical Bending Modes"),
        "img_lateral_modes":  _mode_shape_plot_b64(lateral_freqs,  "Lateral Bending Modes"),
        "img_torsion_modes":  _mode_shape_plot_b64(torsion_freqs,  "Torsional Modes"),
    }


# ---------------------------------------------------------------------------
# API: Single Time-History Simulation (new lightweight endpoint)
# ---------------------------------------------------------------------------

def run_single_simulation(params: dict) -> dict:
    """
    Run one TTB-2D simulation for a single train at a single speed.
    Returns time-history plots (displacement & acceleration at mid-span).

    params keys: bridge_p, track_type, train_name, num_coaches, vel_kmh
    """
    bridge_p   = params["bridge_p"]
    track_type = params.get("track_type", "with_ballast")
    train_name = params.get("train_name", "A1")
    num_coaches = params.get("num_coaches", None)
    vel_kmh    = float(params.get("vel_kmh", 300.0))
    fast_mode  = bool(params.get("fast_mode", False))

    # In fast mode, cap default HSLM consist length unless user explicitly sets coach count.
    if fast_mode and num_coaches is None and train_name in HSLM_PARAMS:
        num_coaches = min(8, HSLM_PARAMS[train_name][0])

    # Build train
    custom_params = params.get("custom_train_params", None)
    if train_name == "Custom" and custom_params:
        from ttb2d.train_properties import TrainProp_Custom
        n_c = num_coaches if num_coaches is not None else 10
        veh_list = [TrainProp_Custom(
            m_body=custom_params.get("m_body", 40000),
            L_body=custom_params.get("L_body", 15.0),
            m_bogie=custom_params.get("m_bogie", 3000),
            m_wheel=custom_params.get("m_wheel", 1500)
        ) for _ in range(n_c)]
    else:
        veh_list = TrainProp_HSLM(train_name, num_coaches)

    Train = types.SimpleNamespace()
    Train.vel = vel_kmh / 3.6
    Train.Veh = types.SimpleNamespace()
    Train.Veh.data = veh_list
    Train.Veh.num = len(veh_list)

    Track = _make_track(track_type)
    Track.Rail.Mesh.Ele.num_per_spacing = 1

    Beam = _make_beam(bridge_p)
    Calc = _make_calc(bridge_p.get("profile"), fast_mode=fast_mode)

    # Run full simulation
    Calc, Train, Track, Beam, Model, Sol = B00_Calculations(Calc, Train, Track, Beam)

    # Extract time array
    t = Calc.Solver.t  # shape (n_steps,)

    # Extract mid-span displacement [m -> mm] and acceleration [m/s²]
    L = float(bridge_p["L"])

    # Get mid-span node index (closest node to L/2)
    n_beam_nodes = Sol.Beam.U.xt.shape[0]
    if (hasattr(Beam, "Mesh") and hasattr(Beam.Mesh, "Nodes")
            and hasattr(Beam.Mesh.Nodes, "coord")):
        coords = Beam.Mesh.Nodes.coord
        mid_node = int(np.argmin(np.abs(coords - L / 2.0)))
    else:
        mid_node = n_beam_nodes // 2

    disp_mm = Sol.Beam.U.xt[mid_node, :] * 1000.0    # mm
    acc_ms2 = Sol.Beam.Acc.xt[mid_node, :]            # m/s²

    max_disp_mm = float(np.max(np.abs(disp_mm)))
    max_acc_ms2 = float(np.max(np.abs(acc_ms2)))
    duration_s  = float(t[-1])

    # Downsample plotting only (keep statistics on full arrays).
    plot_stride = 1
    if fast_mode and len(t) > 1200:
        plot_stride = int(np.ceil(len(t) / 1200))
    t_plot = t[::plot_stride]
    disp_plot = disp_mm[::plot_stride]
    acc_plot = acc_ms2[::plot_stride]

    plot_dpi = 90 if fast_mode else 120

    # ── Plot 1: Displacement time-history ──────────────────────────────────
    COLOR_DISP = "#00d4ff"
    COLOR_ACC  = "#ff6b6b"
    DARK_BG    = "#1a1a2e"
    PANEL_BG   = "#16213e"

    fig_disp, ax_d = plt.subplots(figsize=(9, 4))
    fig_disp.patch.set_facecolor(DARK_BG)
    ax_d.set_facecolor(PANEL_BG)
    ax_d.plot(t_plot, disp_plot, color=COLOR_DISP, linewidth=1.5, label=f"Chuyển vị giữa nhịp (max = {max_disp_mm:.2f} mm)")
    ax_d.axhline(0, color="#555", linewidth=0.8)
    ax_d.set_xlabel("Thời gian t (s)", color="#ccc", fontsize=11)
    ax_d.set_ylabel("Chuyển vị (mm)", color="#ccc", fontsize=11)
    ax_d.set_title(
        f"Chuyển vị giữa nhịp — {train_name} @ {vel_kmh:.0f} km/h",
        color="white", fontsize=13, fontweight="bold", pad=10
    )
    ax_d.tick_params(colors="#aaa")
    ax_d.spines[:].set_color("#333")
    ax_d.grid(True, color="#2a2a4a", linewidth=0.7, linestyle="--")
    legend = ax_d.legend(fontsize=9, facecolor="#0d1117", labelcolor="#ccc", edgecolor="#333")
    fig_disp.tight_layout(pad=1.5)
    img_disp_time = _fig_to_b64(fig_disp, dpi=plot_dpi)

    # ── Plot 2: Acceleration time-history ──────────────────────────────────
    fig_acc, ax_a = plt.subplots(figsize=(9, 4))
    fig_acc.patch.set_facecolor(DARK_BG)
    ax_a.set_facecolor(PANEL_BG)
    ax_a.plot(t_plot, acc_plot, color=COLOR_ACC, linewidth=1.5, label=f"Gia tốc giữa nhịp (max = {max_acc_ms2:.3f} m/s²)")
    ax_a.axhline(0, color="#555", linewidth=0.8)
    ax_a.set_xlabel("Thời gian t (s)", color="#ccc", fontsize=11)
    ax_a.set_ylabel("Gia tốc (m/s²)", color="#ccc", fontsize=11)
    ax_a.set_title(
        f"Gia tốc giữa nhịp — {train_name} @ {vel_kmh:.0f} km/h",
        color="white", fontsize=13, fontweight="bold", pad=10
    )
    ax_a.tick_params(colors="#aaa")
    ax_a.spines[:].set_color("#333")
    ax_a.grid(True, color="#2a2a4a", linewidth=0.7, linestyle="--")
    ax_a.legend(fontsize=9, facecolor="#0d1117", labelcolor="#ccc", edgecolor="#333")
    fig_acc.tight_layout(pad=1.5)
    img_acc_time = _fig_to_b64(fig_acc, dpi=plot_dpi)

    # Educational verdict comparison against Eurocode limit (3.5 m/s² for ballast)
    limit_val = 3.5
    verdict_str = "ĐẠT" if max_acc_ms2 <= limit_val else "KHÔNG ĐẠT"
    mode_text = "FAST" if fast_mode else "BALANCED"
    status_text = (
        f"Hoàn thành mô phỏng! Đoàn tàu {train_name} @ {vel_kmh:.0f} km/h.\n"
        f"• Chế độ tính toán: {mode_text}\n"
        f"• Chuyển vị giữa nhịp cực đại: Δ_max = {max_disp_mm:.2f} mm\n"
        f"• Gia tốc sàn dầm cực đại: a_max = {max_acc_ms2:.3f} m/s²\n"
        f"► Đánh giá an toàn dao động (EN 1991-2): {verdict_str} (Giới hạn cho phép: {limit_val} m/s²)"
    )

    return {
        "img_disp_time": img_disp_time,
        "img_acc_time":  img_acc_time,
        "max_disp_mm":   max_disp_mm,
        "max_acc_ms2":   max_acc_ms2,
        "duration_s":    duration_s,
        "status_text":   status_text,
    }


# ---------------------------------------------------------------------------
# Legacy: Dynamic sweep (disabled on cloud – kept for local use)
# ---------------------------------------------------------------------------

def _run_single_job(args: tuple) -> dict:
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
            "train": train_name, "vel_kmh": vel_kmh,
            "disp_min05": float(Sol.Beam.U.min05),
            "BM_max05":   float(Sol.Beam.BM.max05),
            "Acc_max05":  float(Sol.Beam.Acc.max05),
            "contactLost": bool(Sol.contactLost),
        }
        if hasattr(Sol.Beam, "Shear"):
            result["Shear_max"] = float(Sol.Beam.Shear.max)
        return result
    except Exception as exc:
        return {"train": train_name, "vel_kmh": vel_kmh, "error": str(exc)}


def run_dynamic_sweep(params: dict, update_cb=None) -> dict:
    bridge_p   = params["bridge_p"]
    track_type = params.get("track_type", "with_ballast")
    train_names = params.get("train_names", [f"A{i}" for i in range(1, 11)])
    num_coaches = params.get("num_coaches", None)
    v_min  = float(params.get("v_min_kmh", 250))
    v_max  = float(params.get("v_max_kmh", 350))
    v_step = float(params.get("v_step_kmh", 10))

    velocities = np.arange(v_min, v_max + v_step * 0.5, v_step).tolist()
    jobs = [(tn, vel, num_coaches, bridge_p, track_type) for tn in train_names for vel in velocities]
    total = len(jobs)
    results = []
    completed = 0
    max_workers = int(params.get("max_workers", max(1, (os.cpu_count() or 2) - 1)))

    if update_cb:
        update_cb(0.01, f"Đang chuẩn bị {total} tổ hợp tính toán...")

    from concurrent.futures import ProcessPoolExecutor, as_completed
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = {executor.submit(_run_single_job, job): job for job in jobs}
        for fut in as_completed(futures):
            res = fut.result()
            results.append(res)
            completed += 1
            if update_cb:
                update_cb(max(0.01, min(0.99, completed / total)),
                          f"Hoàn thành {completed}/{total} – {res.get('train','?')} @ {res.get('vel_kmh',0):.0f} km/h")

    valid_results = [r for r in results if "error" not in r]
    if not valid_results:
        raise RuntimeError("All sweep jobs failed.")

    freqs_hz = _beam_natural_frequencies_hz(bridge_p, n_modes=3)
    figs = C04_HSLM_Summary_Plots(
        results=results, eurocode_acc_limit=3.5,
        beam_natural_freq_hz=freqs_hz,
        excitation_lengths_m=np.array([18.0, 27.0]),
    )
    return {
        "results":    results,
        "img_disp":   _fig_to_b64(figs["fig_disp"]),
        "img_acc":    _fig_to_b64(figs["fig_acc"]),
        "img_worst":  _fig_to_b64(figs["fig_worst"]),
        "img_freq":   _fig_to_b64(figs["fig_freq"]),
    }
