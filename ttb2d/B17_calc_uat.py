"""
B17 - Calculate Beam Deformation Under Wheels.
Extracts the vertical displacement of the beam at each wheel location using shape functions.
"""
import numpy as np
from .B03_beam_matrices import shape_fun


def B17_CalcUat(x, Beam, u):
    """Return the vertical displacement at position x from beam DOF vector u."""
    acum = Beam.Mesh.Nodes.acum
    tol = 1e-10

    if x < acum[0] - tol or x > acum[-1] + tol:
        return 0.0

    x = max(x, acum[0])
    x = min(x, acum[-1])

    ele_idx = np.searchsorted(acum[1:], x, side='left')
    ele_idx = min(ele_idx, Beam.Mesh.Ele.Tnum - 1)

    x_e = x - acum[ele_idx]
    a = Beam.Mesh.Ele.a[ele_idx]

    N = shape_fun(x_e, a).flatten()
    dofs = Beam.Mesh.Ele.DOF[ele_idx]

    if hasattr(u, 'toarray'):
        u = np.asarray(u.toarray()).flatten()
    elif hasattr(u, 'A'):
        u = np.asarray(u.A).flatten()
    else:
        u = np.asarray(u).flatten()

    return float(N @ u[dofs])
