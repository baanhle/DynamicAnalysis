"""
B25 - Wheel Profiles
Assigns track irregularity profile values to each wheel for each time step.
"""
import numpy as np
from scipy.interpolate import PchipInterpolator
import types


def B25_WheelProfiles(Calc, Veh_list):
    num_veh = Veh_list[0].Tnum if hasattr(Veh_list[0], 'Tnum') else len(Veh_list)

    for veh_num in range(num_veh):
        veh = Veh_list[veh_num]
        n_wheels = veh.Wheels.num
        num_t = Calc.Solver.num_t

        if not hasattr(Calc, 'Veh') or len(Calc.Veh) <= veh_num:
            if not hasattr(Calc, 'Veh'):
                Calc.Veh = []
            while len(Calc.Veh) <= veh_num:
                Calc.Veh.append(types.SimpleNamespace())

        cv = Calc.Veh[veh_num]
        cv.x_path = np.zeros((n_wheels, num_t))
        cv.h_path = np.zeros((n_wheels, num_t))

        t0 = getattr(Calc.Time, 't_0_ind', 0)
        tend = getattr(Calc.Time, 't_end_ind', num_t)

        x_pos = Calc.Position.x[t0:tend]
        if len(x_pos) < num_t:
            x_pos = Calc.Position.x[:num_t]

        pchip = PchipInterpolator(Calc.Profile.x, Calc.Profile.h)

        for wheel in range(n_wheels):
            cv.x_path[wheel, :] = (x_pos
                                   - veh.Ax_dist[wheel]
                                   - veh.First_wheel_dist)

            x_w = cv.x_path[wheel, :]
            # Clamp to profile range
            x_clamped = np.clip(x_w, Calc.Profile.x[0], Calc.Profile.x[-1])
            cv.h_path[wheel, :] = pchip(x_clamped)

            # Set out-of-range to boundary values
            cv.h_path[wheel, x_w < Calc.Profile.x[0]] = Calc.Profile.h[0]
            cv.h_path[wheel, x_w > Calc.Profile.x[-1]] = Calc.Profile.h[-1]

        # First derivative in time
        cv.hd_path = np.diff(cv.h_path, n=1, axis=1) / Calc.Solver.dt
        cv.hd_path = np.column_stack([cv.hd_path[:, 0], cv.hd_path])

        # Second derivative in time
        cv.hdd_path = np.diff(cv.hd_path, n=1, axis=1) / Calc.Solver.dt
        cv.hdd_path = np.column_stack([cv.hdd_path[:, 0], cv.hdd_path])

        # First point of profile for each wheel at level zero
        cv.h_path = cv.h_path - cv.h_path[:, 0:1]

    return Calc
