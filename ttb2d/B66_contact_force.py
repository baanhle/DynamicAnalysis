"""
B66 - Contact Force
Calculates the vertical contact forces at each wheel.
"""
import numpy as np
import types
from .B17_calc_uat import B17_CalcUat
from .B03_beam_matrices import shape_fun, shape_fun_p, shape_fun_pp


def _calc_deformation_under_wheels(Sol, Track, Calc, Veh_list):
    """Calculate deformation and its derivatives under each wheel at each time step."""
    num_veh = Veh_list[0].Tnum if hasattr(Veh_list[0], 'Tnum') else len(Veh_list)
    num_t = Calc.Solver.num_t
    vel = Veh_list[0].vel

    for veh_num in range(num_veh):
        veh = Veh_list[veh_num]
        cv = Calc.Veh[veh_num]
        n_wheels = veh.Wheels.num

        def_under = np.zeros((n_wheels, num_t))
        vel_under = np.zeros((n_wheels, num_t))
        acc_under = np.zeros((n_wheels, num_t))
        def_under_p = np.zeros((n_wheels, num_t))
        vel_under_p = np.zeros((n_wheels, num_t))
        def_under_pp = np.zeros((n_wheels, num_t))

        for t_step in range(num_t):
            for wheel in range(n_wheels):
                ele_num = cv.elexj[wheel, t_step]
                if ele_num < 0:
                    continue

                x = cv.xj[wheel, t_step]
                a = Track.Rail.Mesh.Ele.a[ele_num]
                dofs = Track.Rail.Mesh.Ele.DOF[ele_num, :]

                sfx = shape_fun(x, a).flatten()
                sfxp = shape_fun_p(x, a).flatten()
                sfxpp = shape_fun_pp(x, a).flatten()

                u_rail = Sol.Model.Nodal.U[dofs, t_step]
                v_rail = Sol.Model.Nodal.V[dofs, t_step]
                a_rail = Sol.Model.Nodal.A[dofs, t_step]

                def_under[wheel, t_step] = sfx @ u_rail
                vel_under[wheel, t_step] = sfx @ v_rail
                acc_under[wheel, t_step] = sfx @ a_rail
                def_under_p[wheel, t_step] = sfxp @ u_rail
                vel_under_p[wheel, t_step] = sfxp @ v_rail
                def_under_pp[wheel, t_step] = sfxpp @ u_rail

        Sol.Veh[veh_num].def_under = def_under
        Sol.Veh[veh_num].vel_under = vel_under
        Sol.Veh[veh_num].acc_under = acc_under
        Sol.Veh[veh_num].def_under_p = def_under_p
        Sol.Veh[veh_num].vel_under_p = vel_under_p
        Sol.Veh[veh_num].def_under_pp = def_under_pp

    return Sol


def B66_ContactForce(Sol, Track, Calc, Train):
    Veh_list = Train.Veh.data
    num_veh = Veh_list[0].Tnum if hasattr(Veh_list[0], 'Tnum') else len(Veh_list)
    num_t = Calc.Solver.num_t
    ones_t = np.ones((1, num_t))
    vel = Train.vel

    if Calc.Options.VBI == 1:
        # Calculate deformations under wheels
        Sol = _calc_deformation_under_wheels(Sol, Track, Calc, Veh_list)

        for veh_num in range(num_veh):
            veh = Veh_list[veh_num]
            cv = Calc.Veh[veh_num]
            sv = Sol.Veh[veh_num]

            ind = (cv.x_path >= 0).astype(float)

            kp = veh.ktn.reshape(-1, 1)
            cp = veh.ctn.reshape(-1, 1)
            mw = veh.mtn.reshape(-1, 1)

            sv.F_onBeam = (
                ((veh.Wheels.N2w @ sv.U) - sv.def_under - cv.h_path * ind) * (kp * ones_t) +
                ((veh.Wheels.N2w @ sv.V) - sv.vel_under - sv.def_under_p * vel
                 - cv.hd_path * ind) * (cp * ones_t) -
                (sv.acc_under + sv.def_under_pp * vel ** 2 +
                 2 * sv.vel_under_p * vel) * (mw * ones_t) +
                (mw * ones_t) * Calc.Cte.grav
            )

            sv.F_onBeam_max = np.max(sv.F_onBeam)
            sv.F_onBeam_min = np.min(sv.F_onBeam)

            # Check contact on bridge
            L1 = Calc.Profile.L_Approach + Calc.Position.x_0 * Calc.Options.redux_factor
            L2 = L1 + Calc.Profile.L_bridge
            bridge_mask = (cv.x_path >= L1) & (cv.x_path <= L2)
            bridge_forces = sv.F_onBeam * bridge_mask
            if np.any(bridge_mask):
                sv.F_onBridge_max = np.max(sv.F_onBeam[bridge_mask])
                sv.F_onBridge_min = np.min(sv.F_onBeam[bridge_mask])
            else:
                sv.F_onBridge_max = 0
                sv.F_onBridge_min = 0
            sv.contactLost = int(sv.F_onBridge_max > 0)

    elif Calc.Options.VBI == 0:
        for veh_num in range(num_veh):
            veh = Veh_list[veh_num]
            cv = Calc.Veh[veh_num]
            sv = Sol.Veh[veh_num]

            kp = veh.ktn.reshape(-1, 1)
            cp = veh.ctn.reshape(-1, 1)
            mw = veh.mtn.reshape(-1, 1)

            sv.F_onBeam = (
                (veh.Wheels.N2w @ sv.U) * (kp * ones_t) +
                (veh.Wheels.N2w @ sv.V) * (cp * ones_t) +
                (mw * ones_t) * Calc.Cte.grav
            )

            sv.F_onBeam_max = np.max(sv.F_onBeam)
            sv.F_onBeam_min = np.min(sv.F_onBeam)

            L1 = Calc.Profile.L_Approach
            L2 = L1 + Calc.Profile.L_bridge
            bridge_mask = (cv.x_path >= L1) & (cv.x_path <= L2)
            sv.contactLost = int(np.max(sv.F_onBeam * bridge_mask) > 0)

    # Check overall contact
    Sol.contactLost = int(max(Sol.Veh[v].contactLost for v in range(num_veh)) > 0)
    if Sol.contactLost:
        print('There is no permanent contact between wheels and rail')

    return Sol
