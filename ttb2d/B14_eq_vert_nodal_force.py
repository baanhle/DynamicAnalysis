"""
B14 - Equivalent Vertical Nodal Force.
Maps a point load at position x to element DOF contributions via shape functions.
"""
import numpy as np
from scipy import sparse
from .B03_beam_matrices import shape_fun


def B14_EqVertNodalForce(x, force_val, Beam):
    """Return a sparse column vector of equivalent nodal forces."""
    ndof = Beam.Mesh.DOF.Tnum
    F = np.zeros(ndof)

    acum = Beam.Mesh.Nodes.acum
    ele_a = Beam.Mesh.Ele.a

    on_beam = (x >= acum[0]) & (x <= acum[-1])
    if not on_beam:
        return sparse.csc_matrix((ndof, 1))

    # Find element
    ele_idx = np.searchsorted(acum[1:], x, side='left')
    ele_idx = min(ele_idx, Beam.Mesh.Ele.Tnum - 1)

    x_e = x - acum[ele_idx]  # local coordinate
    a = ele_a[ele_idx]

    N = shape_fun(x_e, a)
    dofs = Beam.Mesh.Ele.DOF[ele_idx]
    F[dofs] += force_val * N.flatten()

    return sparse.csc_matrix(F.reshape(-1, 1))
