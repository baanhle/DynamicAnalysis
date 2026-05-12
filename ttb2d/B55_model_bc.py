"""
B55 - Model Boundary Conditions
Applies BCs to the coupled Track+Beam model.
"""
import numpy as np
from scipy import sparse


def B55_ModelBC(Model, Beam, Track):
    if not hasattr(Model, 'BC'):
        import types
        Model.BC = types.SimpleNamespace()

    # From Beam DOF to Model DOF
    beam_fixed = Model.Mesh.DOF.beam[Beam.BC.DOF_fixed]
    # From Rail DOF to Model DOF
    rail_fixed = Model.Mesh.DOF.rail[Track.Rail.BC.DOF_fixed]

    Model.BC.DOF_fixed = np.sort(np.concatenate([beam_fixed, rail_fixed])).astype(int)
    Model.BC.num_DOF_fixed = len(Model.BC.DOF_fixed)
    Model.BC.DOF_fixed_value = Beam.BC.DOF_fixed_value

    # Apply fixed DOFs - convert to lil for efficient row/col zeroing
    Mg = Model.Mesh.Mg.tolil()
    Cg = Model.Mesh.Cg.tolil()
    Kg = Model.Mesh.Kg.tolil()

    for d in Model.BC.DOF_fixed:
        Mg[d, :] = 0; Mg[:, d] = 0
        Cg[d, :] = 0; Cg[:, d] = 0
        Kg[d, :] = 0; Kg[:, d] = 0
        Mg[d, d] = Model.BC.DOF_fixed_value
        Kg[d, d] = Model.BC.DOF_fixed_value

    Model.Mesh.Mg = sparse.csc_matrix(Mg)
    Model.Mesh.Cg = sparse.csc_matrix(Cg)
    Model.Mesh.Kg = sparse.csc_matrix(Kg)

    return Model
