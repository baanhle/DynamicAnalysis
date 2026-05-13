"""
Train property definitions.
Each function populates a vehicle SimpleNamespace object.
"""
import numpy as np
import types


def _ns():
    return types.SimpleNamespace()


def _make_veh():
    """Create a vehicle namespace with the standard hierarchy."""
    v = _ns()
    v.Body = _ns()
    v.Bogie = _ns()
    v.Wheels = _ns()
    v.Susp = _ns()
    v.Susp.Prim = _ns()
    v.Susp.Sec = _ns()
    v.Prop = _ns()  # Will hold Ls, Lt etc for B18
    v.DOF = _ns()
    v.Modal = _ns()
    return v


def _finalize_veh(v):
    """Set derived properties needed by solver: Prop.Ls, Prop.Lt, etc."""
def _finalize_veh(v):
    """Ensure all arrays are numpy and tnum set. N2w is built by B18_TrainVehEq."""
    v.Tnum_DOF = 6
    v.Wheels.num = int(v.Wheels.num)
    v.Susp.Prim.k = np.atleast_1d(v.Susp.Prim.k).astype(float)
    v.Susp.Prim.c = np.atleast_1d(v.Susp.Prim.c).astype(float)
    v.Susp.Sec.k = np.atleast_1d(v.Susp.Sec.k).astype(float)
    v.Susp.Sec.c = np.atleast_1d(v.Susp.Sec.c).astype(float)
    v.Wheels.m = np.atleast_1d(v.Wheels.m).astype(float)
    v.Bogie.m = np.atleast_1d(v.Bogie.m).astype(float)
    v.Bogie.I = np.atleast_1d(v.Bogie.I).astype(float)
    v.Bogie.L = np.atleast_1d(v.Bogie.L).astype(float)
    v.Body.Le = np.atleast_1d(v.Body.Le).astype(float)
    # Placeholder N2w (overwritten by B18_TrainVehEq)
    v.Wheels.N2w = np.zeros((4, 6))
    return v


def TrainProp_Manchester_BenchMark():
    """Manchester Benchmark vehicle (Iwnick, 1998)."""
    v = _make_veh()
    v.Body.m = 32000
    v.Body.I = 1970000
    v.Body.L = 9.5 * 2
    v.Body.Le = np.array([1, 1]) * (1.5 + 3 / 2)
    v.Bogie.num = 2
    v.Bogie.m = np.array([1, 1]) * 2615.0
    v.Bogie.I = np.array([1, 1]) * 1476.0
    v.Bogie.L = np.array([1, 1]) * 1.28 * 2
    v.Wheels.num = 4
    v.Wheels.m = np.array([1, 1, 1, 1]) * 1813.0
    v.Susp.Prim.k = np.array([1, 1, 1, 1]) * 1200e3 * 2
    v.Susp.Prim.c = np.array([1, 1, 1, 1]) * 4e3 * 2
    v.Susp.Sec.k = np.array([1, 1]) * 430e3 * 2
    v.Susp.Sec.c = np.array([1, 1]) * 20e3 * 2
    return _finalize_veh(v)


def _hslm_vehicle(N, D, d, P):
    """Generate HSLM-A vehicle per EN 1991-2 parameters."""
    v = _make_veh()
    m_total = (4 * P * 1e3) / 9.81
    v.Body.m = m_total * 0.70
    v.Body.I = v.Body.m * (D ** 2 + 3 ** 2) / 12
    v.Body.L = D - 3
    v.Body.Le = np.array([1.5, 1.5])
    v.Bogie.num = 2
    v.Bogie.m = np.array([1, 1]) * (m_total * 0.15 / 2)
    v.Bogie.I = np.array([1, 1]) * (v.Bogie.m[0] * (d ** 2 + 2.5 ** 2) / 12)
    v.Bogie.L = np.array([1, 1]) * d
    v.Wheels.num = 4
    v.Wheels.m = np.array([1, 1, 1, 1]) * (m_total * 0.15 / 4)
    v.Susp.Prim.k = np.array([1, 1, 1, 1]) * 1200e3 * 2
    v.Susp.Prim.c = np.array([1, 1, 1, 1]) * 4e3 * 2
    v.Susp.Sec.k = np.array([1, 1]) * 430e3 * 2
    v.Susp.Sec.c = np.array([1, 1]) * 20e3 * 2
    return _finalize_veh(v)


# HSLM-A1 to A10 parameters: (N, D[m], d[m], P[kN])
HSLM_PARAMS = {
    'A1':  (18, 18, 2.0, 170),
    'A2':  (17, 19, 3.5, 200),
    'A3':  (16, 20, 2.0, 180),
    'A4':  (15, 21, 3.0, 190),
    'A5':  (14, 22, 2.0, 170),
    'A6':  (13, 23, 2.0, 180),
    'A7':  (13, 24, 2.0, 190),
    'A8':  (12, 25, 2.5, 190),
    'A9':  (11, 26, 2.0, 210),
    'A10': (11, 27, 2.0, 210),
}


def TrainProp_EMU():
    """Electric Multiple Unit (EMU) train - Chinese CR400 AF/BF high-speed train (250-350 km/h)."""
    v = _make_veh()
    # CR400: 8-coach train with axle load 17t, total mass ~68t/coach
    v.Body.m = 50000.0  # kg (car body)
    v.Body.I = 2850000.0  # kg·m²
    v.Body.L = 26.0  # m (coach length: 209m / 8 coaches)
    v.Body.Le = np.array([1.5, 1.5])
    v.Bogie.num = 2
    v.Bogie.m = np.array([1, 1]) * 5500.0  # kg (each bogie)
    v.Bogie.I = np.array([1, 1]) * 8000.0
    v.Bogie.L = np.array([1, 1]) * 2.5
    v.Wheels.num = 4
    v.Wheels.m = np.array([1, 1, 1, 1]) * 2000.0  # kg (each wheel, 17t/4 wheels ≈ 4.25t, but ~2t actual mass)
    v.Susp.Prim.k = np.array([1, 1, 1, 1]) * 1200e3 * 2  # Primary suspension (wheel-bogie)
    v.Susp.Prim.c = np.array([1, 1, 1, 1]) * 40e3 * 2
    v.Susp.Sec.k = np.array([1, 1]) * 500e3 * 2  # Secondary suspension (bogie-body)
    v.Susp.Sec.c = np.array([1, 1]) * 50e3 * 2
    return _finalize_veh(v)


def TrainProp_HSLM(train_name, num_coaches=None):
    """
    Router to create a list of vehicle namespaces. Supports HSLM-A1 to A10,
    ChineseStar, ShinkansenS300, EMU, and Custom.
    """
    if train_name == "EMU":
        n_c = num_coaches if num_coaches is not None else 1
        return [TrainProp_EMU() for _ in range(n_c)]
    elif train_name == "ChineseStar":
        n_c = num_coaches if num_coaches is not None else 1
        return [TrainProp_ChineseStarPowerCar() for _ in range(n_c)]
    elif train_name == "ShinkansenS300":
        n_c = num_coaches if num_coaches is not None else 1
        return [TrainProp_ShinkansenS300() for _ in range(n_c)]
    elif train_name == "Custom":
        n_c = num_coaches if num_coaches is not None else 1
        return [TrainProp_Custom() for _ in range(n_c)]
    elif train_name in HSLM_PARAMS:
        N, D, d, P = HSLM_PARAMS[train_name]
        if num_coaches is None:
            num_coaches = N
        vehicles = []
        for _ in range(num_coaches):
            vehicles.append(_hslm_vehicle(N, D, d, P))
        return vehicles
    else:
        # Fallback to A1
        N, D, d, P = HSLM_PARAMS['A1']
        return [_hslm_vehicle(N, D, d, P)]


def TrainProp_ChineseStarPowerCar():
    """Chinese Star power car (Zhai et al., 2009)."""
    v = _make_veh()
    v.Body.m = 59364.2
    v.Body.I = 1.723e6
    v.Body.L = 5.73 * 2
    v.Body.Le = np.array([1, 1]) * (1.5 + 3 / 2)
    v.Bogie.num = 2
    v.Bogie.m = np.array([1, 1]) * 5630.8
    v.Bogie.I = np.array([1, 1]) * 9487.0
    v.Bogie.L = np.array([1, 1]) * 1.5 * 2
    v.Wheels.num = 4
    v.Wheels.m = np.array([1, 1, 1, 1]) * 1843.5
    v.Susp.Prim.k = np.array([1, 1, 1, 1]) * 2.3996e6 * 2
    v.Susp.Prim.c = np.array([1, 1, 1, 1]) * 30e3 * 2
    v.Susp.Sec.k = np.array([1, 1]) * 0.8858e6 * 2
    v.Susp.Sec.c = np.array([1, 1]) * 45e3 * 2
    return _finalize_veh(v)


def TrainProp_ShinkansenS300():
    """Shinkansen S300 (Wu and Yang, 2003)."""
    v = _make_veh()
    v.Body.m = 41750.0
    v.Body.I = 2080000.0
    v.Body.L = 17.5
    v.Body.Le = np.array([1, 1]) * (25 - 17.5) / 2
    v.Bogie.num = 2
    v.Bogie.m = np.array([1, 1]) * 3040.0
    v.Bogie.I = np.array([1, 1]) * 3930.0
    v.Bogie.L = np.array([1, 1]) * 2.5
    v.Wheels.num = 4
    v.Wheels.m = np.array([1, 1, 1, 1]) * 1780.0
    v.Susp.Prim.k = np.array([1, 1, 1, 1]) * 1180e3
    v.Susp.Prim.c = np.array([1, 1, 1, 1]) * 39.2e3
    v.Susp.Sec.k = np.array([1, 1]) * 530e3
    v.Susp.Sec.c = np.array([1, 1]) * 90.2e3
    return _finalize_veh(v)


def TrainProp_Custom(m_body=40000, L_body=15.0, m_bogie=3000, m_wheel=1500):
    """Generic customizable train configuration for research setup."""
    v = _make_veh()
    v.Body.m = float(m_body)
    v.Body.I = v.Body.m * (L_body ** 2 + 3 ** 2) / 12
    v.Body.L = float(L_body)
    v.Body.Le = np.array([1.5, 1.5])
    v.Bogie.num = 2
    v.Bogie.m = np.array([1, 1]) * float(m_bogie)
    v.Bogie.I = np.array([1, 1]) * 4000.0
    v.Bogie.L = np.array([1, 1]) * 2.5
    v.Wheels.num = 4
    v.Wheels.m = np.array([1, 1, 1, 1]) * float(m_wheel)
    v.Susp.Prim.k = np.array([1, 1, 1, 1]) * 1200e3 * 2
    v.Susp.Prim.c = np.array([1, 1, 1, 1]) * 4e3 * 2
    v.Susp.Sec.k = np.array([1, 1]) * 430e3 * 2
    v.Susp.Sec.c = np.array([1, 1]) * 20e3 * 2
    return _finalize_veh(v)
