"""
B53 - Beam Acceleration
Extracts beam acceleration from nodal results.
"""
import numpy as np
import types


def B53_BeamAcceleration(Sol, Model, Beam, Calc):
    if not hasattr(Sol, 'Beam'):
        Sol.Beam = types.SimpleNamespace()
    Sol.Beam.Acc = types.SimpleNamespace()

    Sol.Beam.Acc.xt = Sol.Model.Nodal.A[Model.Mesh.DOF.beam_vert, :]

    abs_acc = np.abs(Sol.Beam.Acc.xt)
    max_per_time = np.max(abs_acc, axis=0)
    max_node_per_time = np.argmax(abs_acc, axis=0)
    t_crit = np.argmax(max_per_time)
    Sol.Beam.Acc.max = max_per_time[t_crit]
    Sol.Beam.Acc.COP = Beam.Mesh.Nodes.acum[max_node_per_time[t_crit]]
    Sol.Beam.Acc.pCOP = Sol.Beam.Acc.COP / Beam.Prop.L * 100
    Sol.Beam.Acc.t_crit = Calc.Solver.t[t_crit]

    if hasattr(Beam.Mesh.Nodes, 'Mid') and Beam.Mesh.Nodes.Mid.exists == 1:
        Sol.Beam.Acc.max05 = np.max(np.abs(Sol.Beam.Acc.xt[Beam.Mesh.Nodes.Mid.node, :]))
    else:
        max_vals = np.max(np.abs(Sol.Beam.Acc.xt), axis=1)
        Sol.Beam.Acc.max05 = np.interp(Beam.Prop.L / 2, Beam.Mesh.Nodes.acum, max_vals)

    return Sol
