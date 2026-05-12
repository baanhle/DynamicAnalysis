"""
B24 - Beam Damping Matrix (Rayleigh damping).
"""
import numpy as np
from scipy import sparse


def B24_BeamDamping(Beam):
    per = Beam.Damping.per if hasattr(Beam.Damping, 'per') else 0

    if per > 0:
        n_rigid = Beam.Modal.num_rigid_modes
        wr = Beam.Modal.w[n_rigid:n_rigid + 2]

        A_mat = 0.5 * np.array([[1 / wr[0], wr[0]],
                                 [1 / wr[1], wr[1]]])
        b_vec = np.array([1, 1]) * (per / 100.0)
        coeffs = np.linalg.solve(A_mat, b_vec)

        Kg = Beam.Mesh.Kg
        Mg = Beam.Mesh.Mg
        if sparse.issparse(Kg):
            Kg = Kg.toarray()
        if sparse.issparse(Mg):
            Mg = Mg.toarray()

        Cg = coeffs[0] * Mg + coeffs[1] * Kg
        Beam.Mesh.Cg = sparse.csc_matrix(Cg)
    else:
        n = Beam.Mesh.DOF.Tnum
        Beam.Mesh.Cg = sparse.csc_matrix((n, n))

    return Beam
