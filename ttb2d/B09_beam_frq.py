"""
B09 - Beam Frequency calculation (eigenvalue analysis).
"""
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import eigsh


def B09_BeamFrq(Beam, Calc):
    calc_frq = getattr(Calc.Options, 'calc_beam_frq', 0)
    calc_modes = getattr(Calc.Options, 'calc_beam_modes', 0)

    if calc_frq == 1 and calc_modes == 0:
        K = Beam.Mesh.Kg
        M = Beam.Mesh.Mg
        if sparse.issparse(K):
            K = K.toarray()
        if sparse.issparse(M):
            M = M.toarray()
        lam = np.real(np.linalg.eigvals(np.linalg.solve(M, K)))
        lam = np.sort(lam)
        w = np.sqrt(np.abs(lam))
        f = w / (2 * np.pi)
        n_skip = Beam.BC.num_DOF_fixed
        Beam.Modal.w = w[n_skip:]
        Beam.Modal.f = f[n_skip:]

    elif calc_frq == 1 and calc_modes == 1:
        K = Beam.Mesh.Kg
        M = Beam.Mesh.Mg
        if sparse.issparse(K):
            K = K.toarray()
        if sparse.issparse(M):
            M = M.toarray()
        lam, V = np.linalg.eig(np.linalg.solve(M, K))
        idx = np.argsort(np.real(lam))
        lam = np.real(lam[idx])
        V = np.real(V[:, idx])

        # Normalize
        factor = np.diag(V.T @ M @ V).copy()
        factor[factor <= 0] = 1.0
        Beam.Modal.modes = V / np.sqrt(np.abs(factor))

        w = np.sqrt(np.abs(lam))
        f = w / (2 * np.pi)

        n_skip = Beam.BC.num_DOF_fixed
        Beam.Modal.w = w[n_skip:]
        Beam.Modal.f = f[n_skip:]
        Beam.Modal.modes = Beam.Modal.modes[:, n_skip:]

    return Beam
