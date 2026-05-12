"""
B33 - Beam Shear Force
Calculates shear force from nodal displacements using element HS matrices.
"""
import numpy as np
import types


def _beam_ele_HS(L, E, I):
    """Element-level shear extraction matrix [2x4]."""
    return E * I * np.array([
        [12 / L ** 3, 6 / L ** 2, -12 / L ** 3, 6 / L ** 2],
        [12 / L ** 3, 6 / L ** 2, -12 / L ** 3, 6 / L ** 2]
    ])


def B33_BeamShear(Sol, Model, Beam, Calc, calc_type):
    if calc_type == 0:
        out_field = 'StaticShear'
        in_field = 'StaticU'
    else:
        out_field = 'Shear'
        in_field = 'U'

    if not hasattr(Sol, 'Beam'):
        Sol.Beam = types.SimpleNamespace()
    if not hasattr(Sol.Beam, out_field):
        setattr(Sol.Beam, out_field, types.SimpleNamespace())
    shear = getattr(Sol.Beam, out_field)

    num_t = Calc.Solver.num_t
    n_nodes = Beam.Mesh.Nodes.Tnum
    shear.xt = np.zeros((n_nodes, num_t))

    nodal_data = getattr(Sol.Model.Nodal, in_field)

    if getattr(Calc.Options, 'Shear_calc_mode', 1) == 0:
        for ele in range(Beam.Mesh.Ele.Tnum):
            HS = _beam_ele_HS(Beam.Mesh.Ele.a[ele], Beam.Prop.E_n[ele], Beam.Prop.I_n[ele])
            dofs = Model.Mesh.DOF.beam[Beam.Mesh.Ele.DOF[ele, :]]
            shear.xt[ele, :] = HS[0, :] @ nodal_data[dofs, :]
        ele = Beam.Mesh.Ele.Tnum - 1
        HS = _beam_ele_HS(Beam.Mesh.Ele.a[ele], Beam.Prop.E_n[ele], Beam.Prop.I_n[ele])
        dofs = Model.Mesh.DOF.beam[Beam.Mesh.Ele.DOF[ele, :]]
        shear.xt[n_nodes - 1, :] = HS[1, :] @ nodal_data[dofs, :]
    else:
        for ele in range(Beam.Mesh.Ele.Tnum):
            HS = _beam_ele_HS(Beam.Mesh.Ele.a[ele], Beam.Prop.E_n[ele], Beam.Prop.I_n[ele])
            dofs = Model.Mesh.DOF.beam[Beam.Mesh.Ele.DOF[ele, :]]
            shear.xt[ele:ele + 2, :] += HS @ nodal_data[dofs, :]
        shear.xt[1:-1, :] /= 2

    # Max shear
    max_per_time = np.max(shear.xt, axis=0)
    max_node_per_time = np.argmax(shear.xt, axis=0)
    t_crit = np.argmax(max_per_time)
    shear.max = max_per_time[t_crit]
    shear.max_node = max_node_per_time[t_crit]
    shear.max_COP = Beam.Mesh.Nodes.acum[shear.max_node]
    shear.max_pCOP = shear.max_COP / Beam.Prop.L * 100
    shear.max_t_crit = Calc.Solver.t[t_crit]
    if shear.max_pCOP < 50:
        shear.max_supp = np.max(shear.xt[0, :])
    else:
        shear.max_supp = np.max(shear.xt[-1, :])

    # Min shear
    min_per_time = np.min(shear.xt, axis=0)
    min_node_per_time = np.argmin(shear.xt, axis=0)
    t_crit2 = np.argmin(min_per_time)
    shear.min = min_per_time[t_crit2]
    shear.min_node = min_node_per_time[t_crit2]
    shear.min_COP = Beam.Mesh.Nodes.acum[shear.min_node]
    shear.min_pCOP = shear.min_COP / Beam.Prop.L * 100
    shear.min_t_crit = Calc.Solver.t[t_crit2]
    if shear.min_pCOP < 50:
        shear.min_supp = np.min(shear.xt[0, :])
    else:
        shear.min_supp = np.min(shear.xt[-1, :])

    return Sol
