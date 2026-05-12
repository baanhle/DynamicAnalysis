"""
B49 - Beam Deformation
Extracts beam vertical displacements from nodal results.
"""
import numpy as np
import types


def B49_BeamDeformation(Sol, Model, Beam, Calc, Train, calc_type):
    if calc_type == 0:
        usefield = 'StaticU'
        # For static: need to assemble static forces and solve
        # (simplified approach using Kg\F)
        from .B14_eq_vert_nodal_force import B14_EqVertNodalForce
        from scipy.sparse.linalg import spsolve
        from scipy import sparse

        n_dof = Model.Mesh.DOF.Tnum
        F_total = np.zeros(n_dof)

        Veh_list = Train.Veh.data
        num_veh = len(Veh_list)

        # Simplified: compute static solution using Kg and static loads
        num_t = Calc.Solver.num_t
        n_model = Model.Mesh.DOF.Tnum
        StaticU = np.zeros((n_model, num_t))

        for veh_num in range(num_veh):
            cv = Calc.Veh[veh_num]
            veh = Veh_list[veh_num]
            num_t_moving = cv.elexj.shape[1]
            for t_step in range(num_t_moving):
                F = np.zeros(n_model)
                for wheel in range(veh.Wheels.num):
                    ele_num = cv.elexj[wheel, t_step]
                    if ele_num < 0:
                        continue
                    x = cv.xj[wheel, t_step]
                    a = Model.Mesh.Ele.a[ele_num]
                    from .B03_beam_matrices import shape_fun
                    sfx = shape_fun(x, a).flatten()
                    dofs = Model.Mesh.Ele.DOF[ele_num]
                    F[dofs] += veh.sta_loads[wheel] * sfx

                F[Model.BC.DOF_fixed] = 0
                StaticU[:, t_step] += spsolve(Model.Mesh.Kg, F)

        if not hasattr(Sol.Model.Nodal, 'StaticU'):
            Sol.Model.Nodal.StaticU = StaticU

    elif calc_type == 1:
        usefield = 'U'

    if not hasattr(Sol, 'Beam'):
        Sol.Beam = types.SimpleNamespace()
    if not hasattr(Sol.Beam, usefield):
        setattr(Sol.Beam, usefield, types.SimpleNamespace())

    beam_result = getattr(Sol.Beam, usefield)

    if calc_type == 0:
        beam_result.xt = Sol.Model.Nodal.StaticU[Model.Mesh.DOF.beam_vert, :]
    else:
        beam_result.xt = Sol.Model.Nodal.U[Model.Mesh.DOF.beam_vert, :]

    # Min displacement
    min_per_time = np.min(beam_result.xt, axis=0)
    min_node_per_time = np.argmin(beam_result.xt, axis=0)
    t_crit = np.argmin(min_per_time)
    beam_result.min = min_per_time[t_crit]
    beam_result.COP = Beam.Mesh.Nodes.acum[min_node_per_time[t_crit]]
    beam_result.pCOP = beam_result.COP / Beam.Prop.L * 100
    beam_result.t_crit = Calc.Solver.t[t_crit]

    # Mid-span
    if hasattr(Beam.Mesh.Nodes, 'Mid') and Beam.Mesh.Nodes.Mid.exists == 1:
        beam_result.min05 = np.min(beam_result.xt[Beam.Mesh.Nodes.Mid.node, :])
    else:
        min_vals = np.min(beam_result.xt, axis=1)
        beam_result.min05 = np.interp(Beam.Prop.L / 2, Beam.Mesh.Nodes.acum, min_vals)

    return Sol
