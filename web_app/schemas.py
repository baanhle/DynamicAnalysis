"""
Pydantic schemas for the HSLM Web Application API.
"""
from typing import List, Optional
from pydantic import BaseModel, Field


class ProfileParams(BaseModel):
    type: int = Field(0, description="0: Flat, 1: PSD, 3: Cosine Bump")
    psd_type: str = Field("FRA_6", description="FRA_6, German_L, Chinese_HSR, Eurocode_L, etc.")
    seed: int = Field(0, description="Random seed for PSD generation")
    amp: float = Field(0.001, description="Amplitude for bump (m)")
    length: float = Field(1.0, description="Length for bump (m)")


class BridgeParams(BaseModel):
    L: float = Field(50.0, description="Span length [m]")
    E: float = Field(3.5e10, description="Young's modulus [N/m2]")
    I: float = Field(51.3, description="Second moment of area [m4]")
    Ixx: float = Field(51.3, description="Area inertia about section x-axis [m4]")
    Iyy: float = Field(25.0, description="Area inertia about section y-axis [m4]")
    Ixy: float = Field(0.0, description="Product inertia of section [m4]")
    G: float = Field(1.35e10, description="Shear modulus [N/m2]")
    J: float = Field(5.0, description="Saint-Venant torsional constant [m4]")
    I_theta: float = Field(8.0e4, description="Mass polar inertia per unit length [kg·m]")
    rho: float = Field(69000.0, description="Mass per unit length [kg/m]")
    damping_pct: float = Field(2.0, description="Damping ratio [%]")
    ele_per_spacing: int = Field(2, description="Elements per sleeper spacing")
    track_type: str = Field("with_ballast", description="'with_ballast' or 'no_ballast'")
    profile: ProfileParams = Field(default_factory=ProfileParams)


class VibrationCheckRequest(BaseModel):
    bridge: BridgeParams
    n_modes: int = Field(3, description="Number of modes to compute")


class ModeResult(BaseModel):
    mode: int
    freq_hz: float


class ModeDisplayRow(BaseModel):
    mode: int
    mode_idx: int
    mode_type: str
    freq_hz: float
    img_mode_shape: str


class VibrationCheckResponse(BaseModel):
    vertical_modes: List[ModeResult]
    lateral_modes: List[ModeResult]
    torsion_modes: List[ModeResult]
    mode_rows: List[ModeDisplayRow]
    # Backward-compatible field (same as vertical_modes)
    modes: List[ModeResult]
    verdict: str          # "warning" | "ok"
    verdict_text: str
    governing_f1_hz: float
    img_vertical_modes: str
    img_lateral_modes: str
    img_torsion_modes: str
    f1_hz: float


class SweepRequest(BaseModel):
    bridge: BridgeParams
    train_names: List[str] = Field(
        default=["A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8", "A9", "A10"]
    )
    num_coaches: Optional[int] = Field(None, description="None = use EN 1991-2 default")
    v_min_kmh: float = Field(250.0)
    v_max_kmh: float = Field(350.0)
    v_step_kmh: float = Field(10.0)


class SweepJobResponse(BaseModel):
    job_id: str


class JobStatusResponse(BaseModel):
    job_id: str
    status: str          # "queued" | "running" | "done" | "error"
    progress: float      # 0.0 – 1.0
    status_text: str
    # Only present when done:
    img_disp: Optional[str] = None
    img_acc: Optional[str] = None
    img_worst: Optional[str] = None
    img_freq: Optional[str] = None
    error_msg: Optional[str] = None
