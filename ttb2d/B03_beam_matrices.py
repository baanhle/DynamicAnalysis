"""
B03 - Beam System Matrices (Mass and Stiffness)
Assembles global K and M matrices using Euler-Bernoulli beam elements.
"""
import numpy as np
from scipy import sparse


def _beam_ele_M(rho, A, L):
    return rho * A * L / 420.0 * np.array([
        [156, 22 * L, 54, -13 * L],
        [22 * L, 4 * L ** 2, 13 * L, -3 * L ** 2],
        [54, 13 * L, 156, -22 * L],
        [-13 * L, -3 * L ** 2, -22 * L, 4 * L ** 2]
    ])


def _beam_ele_K(EI, L):
    return EI / L ** 3 * np.array([
        [12, 6 * L, -12, 6 * L],
        [6 * L, 4 * L ** 2, -6 * L, 2 * L ** 2],
        [-12, -6 * L, 12, -6 * L],
        [6 * L, 2 * L ** 2, -6 * L, 4 * L ** 2]
    ])


def shape_fun(x, a):
    x = np.asarray(x, dtype=float)
    a = np.asarray(a, dtype=float)
    N1 = (a + 2 * x) * (a - x) ** 2 / a ** 3
    N2 = x * (a - x) ** 2 / a ** 2
    N3 = x ** 2 * (3 * a - 2 * x) / a ** 3
    N4 = -x ** 2 * (a - x) / a ** 2
    return np.array([N1, N2, N3, N4])


def shape_fun_p(x, a):
    x = np.asarray(x, dtype=float)
    a = np.asarray(a, dtype=float)
    return np.array([
        -(6 * x * (a - x)) / a ** 3,
        1.0 - (x * (4 * a - 3 * x)) / a ** 2,
        (6 * x * (a - x)) / a ** 3,
        -(x * (2 * a - 3 * x)) / a ** 2
    ])


def shape_fun_pp(x, a):
    x = np.asarray(x, dtype=float)
    a = np.asarray(a, dtype=float)
    return np.array([
        (12 * x) / a ** 3 - 6.0 / a ** 2,
        (6 * x) / a ** 2 - 4.0 / a,
        6.0 / a ** 2 - (12 * x) / a ** 3,
        (6 * x) / a ** 2 - 2.0 / a
    ])


def B03_BeamMatrices(Beam):
    n_dof = Beam.Mesh.DOF.Tnum
    Kg = np.zeros((n_dof, n_dof))
    CMM = np.zeros((n_dof, n_dof))

    for e in range(Beam.Mesh.Ele.Tnum):
        dofs = Beam.Mesh.Ele.DOF[e]
        Me = _beam_ele_M(Beam.Prop.rho_n[e], Beam.Prop.A_n[e], Beam.Mesh.Ele.a[e])
        Ke = _beam_ele_K(Beam.Prop.E_n[e] * Beam.Prop.I_n[e], Beam.Mesh.Ele.a[e])
        ix = np.ix_(dofs, dofs)
        Kg[ix] += Ke
        CMM[ix] += Me

    # Additional mass at ends if m2 field exists
    if hasattr(Beam.Prop, 'm2'):
        for e in [0, Beam.Mesh.Ele.Tnum - 1]:
            Me = _beam_ele_M(Beam.Prop.m2 / Beam.Mesh.Ele.a[e], 1.0, Beam.Mesh.Ele.a[e])
            dofs = Beam.Mesh.Ele.DOF[e]
            ix = np.ix_(dofs, dofs)
            CMM[ix] += Me

    k_Mc = Beam.Options.k_Mconsist if hasattr(Beam.Options, 'k_Mconsist') else 1

    if k_Mc != 1:
        LMM = np.zeros((n_dof, n_dof))
        for e in range(Beam.Mesh.Ele.Tnum):
            Me = Beam.Prop.rho_n[e] * Beam.Prop.A_n[e] * Beam.Mesh.Ele.a[e] * np.diag([0.5, 0, 0.5, 0])
            dofs = Beam.Mesh.Ele.DOF[e]
            ix = np.ix_(dofs, dofs)
            LMM[ix] += Me
        Mg = k_Mc * CMM + (1 - k_Mc) * LMM
    else:
        Mg = CMM

    # Apply spring BCs
    for i in range(Beam.BC.num_DOF_with_values):
        d = Beam.BC.DOF_with_values[i]
        Kg[d, d] += Beam.BC.DOF_stiff_values[i]

    # Apply fixed BCs
    for d in Beam.BC.DOF_fixed:
        Kg[d, :] = 0; Kg[:, d] = 0
        Mg[d, :] = 0; Mg[:, d] = 0
        Kg[d, d] = Beam.BC.DOF_fixed_value
        Mg[d, d] = Beam.BC.DOF_fixed_value

    Beam.Mesh.Kg = sparse.csc_matrix(Kg)
    Beam.Mesh.Mg = sparse.csc_matrix(Mg)

    # Store shape functions on beam
    Beam.Mesh.shape_fun = shape_fun
    Beam.Mesh.shape_fun_p = shape_fun_p
    Beam.Mesh.shape_fun_pp = shape_fun_pp

    return Beam
