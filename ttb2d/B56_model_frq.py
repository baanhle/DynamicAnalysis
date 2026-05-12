"""
B56 - Model Frequency calculation (eigenvalue analysis for the coupled model).
"""
import numpy as np
from scipy import sparse
import types


def B56_ModelFrq(Model, Calc):
    if not hasattr(Model, 'Modal'):
        Model.Modal = types.SimpleNamespace()

    calc_frq = getattr(Calc.Options, 'calc_model_frq', 0)
    calc_modes = getattr(Calc.Options, 'calc_model_modes', 0)

    if calc_frq == 1 and calc_modes == 0:
        print('Calculating model frequencies ...')
        K = Model.Mesh.Kg.toarray() if sparse.issparse(Model.Mesh.Kg) else Model.Mesh.Kg
        M = Model.Mesh.Mg.toarray() if sparse.issparse(Model.Mesh.Mg) else Model.Mesh.Mg
        lam = np.real(np.linalg.eigvals(np.linalg.solve(M, K)))
        lam = np.sort(lam)
        Model.Modal.w = np.sqrt(np.abs(lam))
        Model.Modal.f = Model.Modal.w / (2 * np.pi)
        n_skip = Model.BC.num_DOF_fixed
        Model.Modal.w = Model.Modal.w[n_skip:]
        Model.Modal.f = Model.Modal.f[n_skip:]

    elif calc_frq == 1 and calc_modes == 1:
        print('Calculating model modes and frequencies ...')
        K = Model.Mesh.Kg.toarray() if sparse.issparse(Model.Mesh.Kg) else Model.Mesh.Kg
        M = Model.Mesh.Mg.toarray() if sparse.issparse(Model.Mesh.Mg) else Model.Mesh.Mg
        lam, V = np.linalg.eig(np.linalg.solve(M, K))
        idx = np.argsort(np.real(lam))
        lam = np.real(lam[idx])
        V = np.real(V[:, idx])
        factor = np.diag(V.T @ M @ V)
        factor[factor <= 0] = 1.0
        Model.Modal.modes = V / np.sqrt(np.abs(factor))
        Model.Modal.w = np.sqrt(np.abs(lam))
        Model.Modal.f = Model.Modal.w / (2 * np.pi)
        n_skip = Model.BC.num_DOF_fixed
        Model.Modal.w = Model.Modal.w[n_skip:]
        Model.Modal.f = Model.Modal.f[n_skip:]
        Model.Modal.modes = Model.Modal.modes[:, n_skip:]

    return Model
