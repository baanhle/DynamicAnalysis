"""
B02 - Boundary Conditions
Determines fixed DOFs and spring-supported DOFs from support definitions.
"""
import numpy as np


def B02_BoundaryConditions(Beam):
    bc = Beam.BC
    bc.supp_num = len(bc.loc) if hasattr(bc, 'loc') and bc.loc is not None and len(np.atleast_1d(bc.loc)) > 0 else 0

    loc = np.atleast_1d(bc.loc) if bc.supp_num > 0 else np.array([])
    vert = np.atleast_1d(bc.vert_stiff)
    rot = np.atleast_1d(bc.rot_stiff)

    if bc.supp_num > 0:
        acum = Beam.Mesh.Nodes.acum
        loc_ind = []
        for s in range(bc.supp_num):
            idx = np.argmin(np.abs(acum - loc[s]))
            loc_ind.append(idx)
        bc.loc_ind = np.array(loc_ind, dtype=int)
    else:
        bc.loc_ind = np.array([], dtype=int)

    # Fixed vertical DOF (vert_stiff == -1)
    dof_fixed = []
    for i in range(bc.supp_num):
        if vert[i] == -1:
            dof_fixed.append(bc.loc_ind[i] * 2)  # vertical DOF = node*2 (0-based)

    # Fixed rotational DOF (rot_stiff == -1)
    for i in range(bc.supp_num):
        if rot[i] == -1:
            dof_fixed.append(bc.loc_ind[i] * 2 + 1)

    bc.DOF_fixed = np.sort(np.array(dof_fixed, dtype=int))

    # DOF with stiffness values (vert_stiff > 0)
    dof_with_values = []
    stiff_values = []
    for i in range(bc.supp_num):
        if vert[i] > 0:
            dof_with_values.append(bc.loc_ind[i] * 2)
            stiff_values.append(vert[i])
    for i in range(bc.supp_num):
        if rot[i] > 0:
            dof_with_values.append(bc.loc_ind[i] * 2 + 1)
            stiff_values.append(rot[i])

    if len(dof_with_values) > 0:
        idx_sort = np.argsort(dof_with_values)
        bc.DOF_with_values = np.array(dof_with_values, dtype=int)[idx_sort]
        bc.DOF_stiff_values = np.array(stiff_values)[idx_sort]
    else:
        bc.DOF_with_values = np.array([], dtype=int)
        bc.DOF_stiff_values = np.array([])

    bc.num_DOF_fixed = len(bc.DOF_fixed)
    bc.num_DOF_with_values = len(bc.DOF_with_values)

    if not hasattr(Beam, 'Modal'):
        import types
        Beam.Modal = types.SimpleNamespace()
    Beam.Modal.num_rigid_modes = max(0, 2 - bc.num_DOF_fixed)

    bc.DOF_fixed_value = 1e8

    return Beam
