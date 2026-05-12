"""
B31 - Beam Bending Moment
Calculates BM from nodal displacements using element H matrices.
"""
import numpy as np
import types


def _beam_ele_H(L, E, I):
    """Element-level BM extraction matrix [2x4]."""
    return E * I * np.array([
        [-6 / L ** 2, -4 / L, 6 / L ** 2, -2 / L],
        [6 / L ** 2, 2 / L, -6 / L ** 2, 4 / L]
    ])


def B31_BeamBM(Sol, Model, Beam, Calc, calc_type):
    if calc_type == 0:
        out_field = 'StaticBM'
        in_field = 'StaticU'
    else:
        out_field = 'BM'
        in_field = 'U'

    if not hasattr(Sol, 'Beam'):
        Sol.Beam = types.SimpleNamespace()
    if not hasattr(Sol.Beam, out_field):
        setattr(Sol.Beam, out_field, types.SimpleNamespace())
    bm = getattr(Sol.Beam, out_field)

    num_t = Calc.Solver.num_t
    n_nodes = Beam.Mesh.Nodes.Tnum
    bm.xt = np.zeros((n_nodes, num_t))

    nodal_data = getattr(Sol.Model.Nodal, in_field)

    if getattr(Calc.Options, 'BM_calc_mode', 1) == 0:
        for ele in range(Beam.Mesh.Ele.Tnum):
            H = _beam_ele_H(Beam.Mesh.Ele.a[ele], Beam.Prop.E_n[ele], Beam.Prop.I_n[ele])
            dofs = Model.Mesh.DOF.beam[Beam.Mesh.Ele.DOF[ele, :]]
            bm.xt[ele, :] = H[0, :] @ nodal_data[dofs, :]
        # Last node
        ele = Beam.Mesh.Ele.Tnum - 1
        H = _beam_ele_H(Beam.Mesh.Ele.a[ele], Beam.Prop.E_n[ele], Beam.Prop.I_n[ele])
        dofs = Model.Mesh.DOF.beam[Beam.Mesh.Ele.DOF[ele, :]]
        bm.xt[n_nodes - 1, :] = H[1, :] @ nodal_data[dofs, :]
    else:
        for ele in range(Beam.Mesh.Ele.Tnum):
            H = _beam_ele_H(Beam.Mesh.Ele.a[ele], Beam.Prop.E_n[ele], Beam.Prop.I_n[ele])
            dofs = Model.Mesh.DOF.beam[Beam.Mesh.Ele.DOF[ele, :]]
            bm.xt[ele:ele + 2, :] += H @ nodal_data[dofs, :]
        bm.xt[1:-1, :] /= 2

    # Max BM
    max_per_time = np.max(bm.xt, axis=0)
    max_node_per_time = np.argmax(bm.xt, axis=0)
    t_crit = np.argmax(max_per_time)
    bm.max = max_per_time[t_crit]
    bm.COP = Beam.Mesh.Nodes.acum[max_node_per_time[t_crit]]
    bm.pCOP = bm.COP / Beam.Prop.L * 100
    bm.t_crit = Calc.Solver.t[t_crit]

    # Mid-span BM
    if hasattr(Beam.Mesh.Nodes, 'Mid') and Beam.Mesh.Nodes.Mid.exists == 1:
        bm.max05 = np.max(bm.xt[Beam.Mesh.Nodes.Mid.node, :])
    else:
        max_vals = np.max(bm.xt, axis=1)
        bm.max05 = np.interp(Beam.Prop.L / 2, Beam.Mesh.Nodes.acum, max_vals)

    # Left/Right support BM minima
    for k, (field, start, end) in enumerate([
        ('LeftSup', 0, n_nodes // 2),
        ('RightSup', (n_nodes - 1) // 2, n_nodes)
    ]):
        ns = types.SimpleNamespace()
        sub = bm.xt[start:end, :]
        min_per_time = np.min(sub, axis=0)
        min_node_per_time = np.argmin(sub, axis=0)
        t_crit2 = np.argmin(min_per_time)
        ns.min = min_per_time[t_crit2]
        ns.COP = Beam.Mesh.Nodes.acum[min_node_per_time[t_crit2] + start]
        ns.pCOP = ns.COP / Beam.Prop.L * 100
        ns.min05 = np.min(bm.xt[0, :]) if k == 0 else np.min(bm.xt[-1, :])
        setattr(bm, field, ns)

    return Sol
