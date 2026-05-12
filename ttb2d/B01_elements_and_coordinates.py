"""
B01 - Elements and Coordinates
Generates FEM mesh: node coordinates, DOF mapping, element-wise properties.
"""
import numpy as np
import types


def B01_ElementsAndCoordinates(Beam, Calc=None):
    ele_num = Beam.Mesh.Ele.num
    L = Beam.Prop.L

    # Ensure sub-namespaces exist
    if not hasattr(Beam.Mesh, 'Nodes'):
        Beam.Mesh.Nodes = types.SimpleNamespace()
    if not hasattr(Beam.Mesh, 'DOF'):
        Beam.Mesh.DOF = types.SimpleNamespace()

    Beam.Mesh.Ele.a = np.ones(ele_num) * L / ele_num
    Beam.Mesh.Nodes.acum = np.concatenate(([0], np.cumsum(Beam.Mesh.Ele.a)))
    Beam.Mesh.Nodes.coord = Beam.Mesh.Nodes.acum.copy()
    Beam.Mesh.Ele.num_nodes = 2
    Beam.Mesh.Nodes.num_DOF = 2
    Beam.Mesh.Ele.Tnum = ele_num

    nodes = np.column_stack((np.arange(ele_num), np.arange(ele_num) + 1))
    Beam.Mesh.Ele.nodes = nodes

    dof_start = np.arange(0, ele_num * 2, 2)
    Beam.Mesh.Ele.DOF = np.column_stack((dof_start, dof_start + 1,
                                          dof_start + 2, dof_start + 3))
    Beam.Mesh.Nodes.Tnum = ele_num + 1
    Beam.Mesh.DOF.Tnum = Beam.Mesh.Nodes.Tnum * Beam.Mesh.Nodes.num_DOF

    # Element-wise properties
    if np.isscalar(Beam.Prop.E) or (hasattr(Beam.Prop.E, '__len__') and len(np.atleast_1d(Beam.Prop.E)) == 1):
        Beam.Prop.E_n = np.ones(ele_num) * float(np.atleast_1d(Beam.Prop.E)[0])
    if np.isscalar(Beam.Prop.I) or (hasattr(Beam.Prop.I, '__len__') and len(np.atleast_1d(Beam.Prop.I)) == 1):
        Beam.Prop.I_n = np.ones(ele_num) * float(np.atleast_1d(Beam.Prop.I)[0])
    if np.isscalar(Beam.Prop.rho) or (hasattr(Beam.Prop.rho, '__len__') and len(np.atleast_1d(Beam.Prop.rho)) == 1):
        Beam.Prop.rho_n = np.ones(ele_num) * float(np.atleast_1d(Beam.Prop.rho)[0])
    if np.isscalar(Beam.Prop.A) or (hasattr(Beam.Prop.A, '__len__') and len(np.atleast_1d(Beam.Prop.A)) == 1):
        Beam.Prop.A_n = np.ones(ele_num) * float(np.atleast_1d(Beam.Prop.A)[0])

    # Mid-span info
    if Calc is not None:
        tol = Calc.Cte.tol
        min_val = np.min(np.abs(Beam.Mesh.Nodes.acum - L / 2))
        min_ind = np.argmin(np.abs(Beam.Mesh.Nodes.acum - L / 2))
        if not hasattr(Beam.Mesh.Nodes, 'Mid'):
            Beam.Mesh.Nodes.Mid = types.SimpleNamespace()
        if min_val < tol:
            Beam.Mesh.Nodes.Mid.exists = 1
            Beam.Mesh.Nodes.Mid.node = min_ind
            Beam.Mesh.DOF.Mid = types.SimpleNamespace()
            Beam.Mesh.DOF.Mid.vert = Beam.Mesh.Nodes.num_DOF * min_ind  # 0-based vert DOF
        else:
            Beam.Mesh.Nodes.Mid.exists = 0

    return Beam
