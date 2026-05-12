"""
B64 - Coupled Initial Static
Calculates initial static deformation of vehicle+track coupled system.
"""
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import spsolve
import types
from .B03_beam_matrices import shape_fun


def B64_Coupled_InitialStatic(Veh_list, Model, Calc, Track):
    Sol = types.SimpleNamespace()
    Sol.Veh = [types.SimpleNamespace() for _ in range(len(Veh_list))]
    Sol.Model = types.SimpleNamespace()
    Sol.Model.Nodal = types.SimpleNamespace()

    num_veh = Veh_list[0].Tnum if hasattr(Veh_list[0], 'Tnum') else len(Veh_list)
    global_ind_end = Veh_list[-1].global_ind[-1] + 1
    Coup_DOF_Tnum = global_ind_end + Model.Mesh.DOF.Tnum

    Coup_Kg = sparse.lil_matrix((Coup_DOF_Tnum, Coup_DOF_Tnum))
    Coup_F = np.zeros(Coup_DOF_Tnum)

    redux = getattr(Calc.Options, 'redux', 1)

    if redux == 0:
        if Calc.Options.VBI == 1:
            # Vehicles contributions
            for veh_num in range(num_veh):
                veh = Veh_list[veh_num]
                gi = veh.global_ind
                ix = np.ix_(gi, gi)
                Coup_Kg[ix] += veh.SysM.K

                for wheel in range(veh.Wheels.num):
                    ele_num = Calc.Veh[veh_num].elexj[wheel, 0]
                    x = Calc.Veh[veh_num].xj[wheel, 0]
                    a = Track.Rail.Mesh.Ele.a[ele_num]
                    sfx = shape_fun(x, a).flatten()

                    eq_num = global_ind_end + Track.Rail.Mesh.Ele.DOF[ele_num, :]
                    NN = np.outer(sfx, sfx)
                    eqix = np.ix_(eq_num, eq_num)
                    Coup_Kg[eqix] += NN * veh.Susp.Prim.k[wheel]

                    N2w = veh.Wheels.N2w[wheel, :]
                    OffDiag = -np.outer(sfx, N2w) * veh.Susp.Prim.k[wheel]
                    rows = gi
                    cols = eq_num
                    Coup_Kg[np.ix_(rows, cols)] += OffDiag.T
                    Coup_Kg[np.ix_(cols, rows)] += OffDiag

                    Coup_F[cols] += veh.Wheels.m[wheel] * sfx * Calc.Cte.grav

                Coup_F[gi] += veh.SysM.M @ (veh.DOF.vert * Calc.Cte.grav)

            # Track contribution
            sl = slice(global_ind_end, None)
            Coup_Kg[sl, sl] += Model.Mesh.Kg

            # BCs
            DOF_fixed = global_ind_end + Model.BC.DOF_fixed
            for d in DOF_fixed:
                Coup_Kg[d, :] = 0; Coup_Kg[:, d] = 0
                Coup_Kg[d, d] = Model.BC.DOF_fixed_value
            Coup_F[DOF_fixed] = 0

        else:  # VBI == 0 (Moving Force)
            for veh_num in range(num_veh):
                veh = Veh_list[veh_num]
                gi = veh.global_ind
                ix = np.ix_(gi, gi)
                Coup_Kg[ix] += veh.SysM.K
                Coup_F[gi] += veh.SysM.M @ (veh.DOF.vert * Calc.Cte.grav)

            sl = slice(global_ind_end, None)
            Coup_Kg[sl, sl] += Model.Mesh.Kg

        # Solve
        Coup_Kg_csc = sparse.csc_matrix(Coup_Kg)
        Coup_U0 = spsolve(Coup_Kg_csc, Coup_F)

    elif redux == 1:
        Coup_U0 = np.zeros(Coup_DOF_Tnum)
        for veh_num in range(num_veh):
            veh = Veh_list[veh_num]
            Coup_U0[veh.global_ind] = veh.U0

    # Divide results
    for veh_num in range(num_veh):
        veh = Veh_list[veh_num]
        gi = veh.global_ind
        Sol.Veh[veh_num].U0 = Coup_U0[gi]
        Sol.Veh[veh_num].V0 = np.zeros_like(Coup_U0[gi])
        Sol.Veh[veh_num].A0 = np.zeros_like(Coup_U0[gi])

    Sol.Model.Nodal.U0 = Coup_U0[global_ind_end:]
    Sol.Model.Nodal.V0 = np.zeros_like(Sol.Model.Nodal.U0)
    Sol.Model.Nodal.A0 = np.zeros_like(Sol.Model.Nodal.U0)

    return Sol
