"""
B50 - Element Number of Force
Determines which rail element each wheel is on at each time step, and
the relative position within that element.
"""
import numpy as np
import types


def B50_ElementNumOfForce(RailBeam, Calc):
    num_veh = len(Calc.Veh)

    for veh_num in range(num_veh):
        cv = Calc.Veh[veh_num]
        n_wheels = cv.x_path.shape[0]
        num_t = cv.x_path.shape[1]

        cv.elexj = np.zeros((n_wheels, num_t), dtype=int)
        cv.xj = np.zeros((n_wheels, num_t))

        acum = RailBeam.Mesh.Nodes.acum
        n_ele = RailBeam.Mesh.Ele.Tnum

        for wheel in range(n_wheels):
            xp = cv.x_path[wheel, :].copy()
            elexj = np.zeros(num_t, dtype=int)
            xj = np.zeros(num_t)

            # Assign from last element to first (MATLAB style: last matching wins)
            for j in range(n_ele - 1, -1, -1):
                mask = xp >= acum[j]
                elexj[mask] = j  # 0-based element index
                xj[mask] = xp[mask] - acum[j]
                xp[mask] = -1  # mark as assigned

            cv.elexj[wheel, :] = elexj
            cv.xj[wheel, :] = xj

        # Redux model: zero out positions outside track
        if getattr(Calc.Options, 'redux', 0) == 1:
            profile_L = Calc.Profile.L
            mask_out = (cv.x_path < 0) | (cv.x_path > profile_L)
            cv.elexj[mask_out] = -1  # -1 = not on track (0-based convention)
            cv.xj[mask_out] = 0

    return Calc
