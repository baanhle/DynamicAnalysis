"""
B19 - Generate Track Irregularity Profile.
Generates profiles from Power Spectral Density (PSD) using IFFT method.
"""
import numpy as np
import types


def B19_GenerateProfile(Calc, Beam, Train):
    profile_type = getattr(Calc.Profile, 'Type', getattr(Calc.Profile, 'type', 0))
    if profile_type == 0:
        # No irregularity - flat profile
        L_extra = Beam.Prop.L + 10
        x0 = Calc.Position.x[0] - L_extra
        x1 = Calc.Position.x[-1] + L_extra
        Calc.Profile.x = np.array([x0, x1])
        Calc.Profile.h = np.array([0.0, 0.0])
        Calc.Profile.needed_road_l = x1 - x0
        return Calc, Beam, Train

    if profile_type == 1:
        # PSD-based
        return _generate_psd_profile(Calc, Beam, Train)
    elif profile_type == 2:
        # User-defined profile directly
        return Calc, Beam, Train
    elif profile_type == 3:
        # Cosine bump
        return _generate_cosine_profile(Calc, Beam, Train)
    else:
        return Calc, Beam, Train


def _generate_psd_profile(Calc, Beam, Train):
    L_needed = Calc.Position.x[-1] - Calc.Position.x[0] + Beam.Prop.L + 10
    Calc.Profile.needed_road_l = L_needed

    dx = np.min(Beam.Mesh.Ele.a) / 2
    N_pts = int(np.ceil(L_needed / dx))
    if N_pts % 2 != 0:
        N_pts += 1
    x = np.arange(N_pts) * dx

    Omega_min = 2 * np.pi / L_needed
    Omega_max = 2 * np.pi / (2 * dx)
    N_freq = N_pts // 2

    Omega = np.linspace(Omega_min, Omega_max, N_freq)
    dOmega = Omega[1] - Omega[0]

    psd_type = getattr(Calc.Profile, 'PSD_type', 'FRA_6')
    S = _calc_psd(Omega, psd_type)

    rng_seed = getattr(Calc.Profile, 'seed', None)
    if rng_seed is not None and rng_seed > 0:
        rng = np.random.RandomState(int(rng_seed))
    else:
        rng = np.random.RandomState()

    phi = rng.uniform(0, 2 * np.pi, N_freq)
    amp = np.sqrt(2 * S * dOmega)
    profile = np.zeros(N_pts)
    for k in range(N_freq):
        profile += amp[k] * np.cos(Omega[k] * x + phi[k])

    Calc.Profile.x = x
    Calc.Profile.h = profile

    return Calc, Beam, Train


def _generate_cosine_profile(Calc, Beam, Train):
    L_needed = Calc.Position.x[-1] - Calc.Position.x[0] + Beam.Prop.L + 10
    Calc.Profile.needed_road_l = L_needed

    dx = np.min(Beam.Mesh.Ele.a) / 2
    N_pts = int(np.ceil(L_needed / dx))
    x = np.arange(N_pts) * dx

    amp = getattr(Calc.Profile, 'amp', 0.001)
    length = getattr(Calc.Profile, 'length', 1.0)
    start = getattr(Calc.Profile, 'start', Beam.Prop.L / 2 - length / 2)

    profile = np.zeros(N_pts)
    mask = (x >= start) & (x <= start + length)
    profile[mask] = amp / 2 * (1 - np.cos(2 * np.pi * (x[mask] - start) / length))

    Calc.Profile.x = x
    Calc.Profile.h = profile

    return Calc, Beam, Train


def _calc_psd(Omega, psd_type):
    """Calculate PSD values for given spatial frequencies."""
    if isinstance(psd_type, str) and psd_type.startswith('FRA'):
        class_num = int(psd_type.split('_')[1]) if '_' in psd_type else 6
        # FRA PSD (vertical alignment Av), SI units: m^2·rad/m (factor 1e-7 per TTCI/FRA report)
        kv = [0.2095, 0.4190, 0.8380, 1.6764, 3.3528, 6.7056]
        Av = kv[min(class_num - 1, 5)] * 1e-7
        Oc = 0.8246
        Os = 0.4380
        S = Av * Oc ** 2 / ((Omega ** 2 + Oc ** 2) * (Omega ** 2 + Os ** 2))
    elif isinstance(psd_type, str) and (psd_type.startswith('German') or psd_type.startswith('Eurocode')):
        # ERRI / ORE B176 / German / Eurocode Spectrum
        # Ac represents track quality. L=Low irregularity (Good), H=High (Poor)
        class_letter = 'L'
        if 'High' in psd_type or psd_type.endswith('H'): class_letter = 'H'
        
        Ac_dict = {'L': 1.08e-7, 'H': 4.032e-7}
        Ac = Ac_dict.get(class_letter, 1.08e-7)
        Omega_c = 0.8246
        Omega_r = 0.0206
        Omega_s = 0.4380
        S = Ac * Omega_c ** 2 / ((Omega ** 2 + Omega_r ** 2) * (Omega ** 2 + Omega_s ** 2))
    elif isinstance(psd_type, str) and psd_type.startswith('Chinese'):
        # Chinese HSR PSD (TB/T 3352-2014 / Zhai et al.)
        # Often similar form but different coefficients. 
        # Using representative HSR ballastless track coefficients
        A = 2.812e-7  # Representative for HSR Class 1
        Omega_c = 0.8246
        Omega_r = 0.0206
        Omega_s = 0.4380
        S = A * Omega_c ** 2 / ((Omega ** 2 + Omega_r ** 2) * (Omega ** 2 + Omega_s ** 2))
    elif isinstance(psd_type, str) and psd_type.startswith('SNCF'):
        # French SNCF power law
        S = 1.3e-7 * (1.0 / Omega) ** 3.15
    else:
        S = np.zeros_like(Omega)

    return S
