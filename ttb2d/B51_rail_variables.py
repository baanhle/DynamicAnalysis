"""
B51 - Rail Variables
Creates Rail structure with properties analogous to a Beam object for FEM processing.
"""
import numpy as np
import types


def B51_RailVariables(Track, Calc):
    if not hasattr(Track, 'Rail'):
        Track.Rail = types.SimpleNamespace()
    if not hasattr(Track.Rail, 'Prop'):
        Track.Rail.Prop = types.SimpleNamespace()
    if not hasattr(Track.Rail, 'Mesh'):
        Track.Rail.Mesh = types.SimpleNamespace()
    if not hasattr(Track.Rail.Mesh, 'Ele'):
        Track.Rail.Mesh.Ele = types.SimpleNamespace()

    Track.Rail.Prop.L = Calc.Profile.L
    Track.Rail.Prop.A = 1
    Track.Rail.Mesh.Ele.num = round(
        Track.Rail.Prop.L / Track.Sleeper.spacing * Track.Rail.Mesh.Ele.num_per_spacing
    )

    if not hasattr(Track.Rail, 'Options'):
        Track.Rail.Options = types.SimpleNamespace()
    Track.Rail.Options.k_Mconsist = 1

    if not hasattr(Track.Rail, 'BC'):
        Track.Rail.BC = types.SimpleNamespace()

    if getattr(Calc.Options, 'redux', 1) == 0:
        Track.Rail.BC.loc = np.array([])
        Track.Rail.BC.vert_stiff = np.array([])
        Track.Rail.BC.rot_stiff = np.array([])
    elif Calc.Options.redux == 1:
        Track.Rail.BC.loc = np.array([0, Track.Rail.Prop.L])
        Track.Rail.BC.vert_stiff = np.array([-1, -1])
        Track.Rail.BC.rot_stiff = np.array([0, 0])

    return Track
