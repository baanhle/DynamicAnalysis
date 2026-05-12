"""
B54 - Model Matrices
Assembles the coupled Track+Beam system matrices (Mass, Damping, Stiffness).
"""
import numpy as np
from scipy import sparse
import types


def B54_ModelMatrices(Beam, Track, Calc):
    Model = types.SimpleNamespace()
    Model.Mesh = types.SimpleNamespace()
    Model.Mesh.DOF = types.SimpleNamespace()
    Model.Mesh.XLoc = types.SimpleNamespace()
    Model.Mesh.Ele = types.SimpleNamespace()

    # ---- Counting ----
    Track.Sleeper.Tnum = round(Calc.Profile.L / Track.Sleeper.spacing) + 1

    rf = Calc.Options.redux_factor
    Track.Sleeper.num_app = round(
        (Calc.Profile.max_TL * rf + Calc.Profile.L_Approach) / Track.Sleeper.spacing
    )

    if Calc.Profile.extra_L < Calc.Cte.tol:
        Track.Sleeper.num_onbeam = round(Beam.Prop.L / Track.Sleeper.spacing) + 1
        Track.Sleeper.num_aft = round(
            (Calc.Profile.L - (Calc.Profile.max_TL * rf + Calc.Profile.L_Approach +
             Beam.Prop.L + Calc.Profile.extra_L)) / Track.Sleeper.spacing
        )
    else:
        Track.Sleeper.num_onbeam = int(np.floor(Beam.Prop.L / Track.Sleeper.spacing)) + 1
        Track.Sleeper.num_aft = round(
            (Calc.Profile.L - (Calc.Profile.max_TL * rf + Calc.Profile.L_Approach +
             Beam.Prop.L + Calc.Profile.extra_L)) / Track.Sleeper.spacing
        ) + 1

    # ---- DOF indices (0-based) ----
    rail_dof_num = Track.Rail.Mesh.DOF.Tnum
    Model.Mesh.DOF.rail = np.arange(rail_dof_num)
    Model.Mesh.DOF.rail_vert = Model.Mesh.DOF.rail[0::2]
    Model.Mesh.DOF.rail_vert_at_sleepers = (
        np.arange(0, Track.Rail.Mesh.Nodes.Tnum, Track.Rail.Mesh.Ele.num_per_spacing) * 2
    )

    base = rail_dof_num
    Model.Mesh.DOF.sleepers = np.arange(base, base + Track.Sleeper.Tnum)
    Model.Mesh.DOF.sleepers_app = Model.Mesh.DOF.sleepers[:Track.Sleeper.num_app]
    s1 = Track.Sleeper.num_app
    s2 = s1 + Track.Sleeper.num_onbeam
    Model.Mesh.DOF.sleepers_onbeam = Model.Mesh.DOF.sleepers[s1:s2]
    Model.Mesh.DOF.sleepers_aft = Model.Mesh.DOF.sleepers[s2:s2 + Track.Sleeper.num_aft]

    base = Model.Mesh.DOF.sleepers[-1] + 1
    Model.Mesh.DOF.ballast_app = np.arange(base, base + Track.Sleeper.num_app)

    base = Model.Mesh.DOF.ballast_app[-1] + 1 if Track.Sleeper.num_app > 0 else base
    Model.Mesh.DOF.beam = np.arange(base, base + Beam.Mesh.DOF.Tnum)
    Model.Mesh.DOF.beam_vert = Model.Mesh.DOF.beam[0::2]
    Model.Mesh.DOF.beam_vert_under_sleeper = (
        Model.Mesh.DOF.beam_vert[::Beam.Mesh.Ele.num_per_spacing]
    )

    base = Model.Mesh.DOF.beam[-1] + 1
    Model.Mesh.DOF.ballast_aft = np.arange(base, base + Track.Sleeper.num_aft)

    if Track.Sleeper.num_aft == 0:
        Model.Mesh.DOF.Tnum = int(Model.Mesh.DOF.beam[-1]) + 1
    else:
        Model.Mesh.DOF.Tnum = int(Model.Mesh.DOF.ballast_aft[-1]) + 1

    # ---- X locations ----
    Model.Mesh.XLoc.rail_vert = Track.Rail.Mesh.Nodes.acum
    Model.Mesh.XLoc.sleepers = np.arange(len(Model.Mesh.DOF.sleepers)) * Track.Sleeper.spacing
    Model.Mesh.XLoc.ballast_app = np.arange(len(Model.Mesh.DOF.ballast_app)) * Track.Sleeper.spacing
    Model.Mesh.XLoc.beam_vert = (
        Beam.Mesh.Nodes.acum + Calc.Profile.L_Approach + Calc.Profile.max_TL * rf
    )
    if Track.Sleeper.num_aft > 0:
        Model.Mesh.XLoc.ballast_aft = (
            (np.arange(Track.Sleeper.num_aft) + 1) * Track.Sleeper.spacing +
            Calc.Profile.L_Approach + Beam.Prop.L + Calc.Profile.max_TL * rf + Calc.Profile.extra_L
        )
    else:
        Model.Mesh.XLoc.ballast_aft = np.array([])

    # Temporary name change for PadUnderSleeperOnBeam
    if getattr(Track.PadUnderSleeperOnBeam, 'included', 0) == 1:
        if not hasattr(Track, 'BallastOnBeam'):
            Track.BallastOnBeam = types.SimpleNamespace()
            Track.BallastOnBeam.Prop = types.SimpleNamespace()
        Track.BallastOnBeam.Prop.m = 0
        Track.BallastOnBeam.Prop.c = Track.PadUnderSleeperOnBeam.Prop.c
        Track.BallastOnBeam.Prop.k = Track.PadUnderSleeperOnBeam.Prop.k

    # ---- Build global matrices ----
    N = Model.Mesh.DOF.Tnum
    Mg = sparse.lil_matrix((N, N))
    Cg = sparse.lil_matrix((N, N))
    Kg = sparse.lil_matrix((N, N))

    def funAdd1(mat, ind, add_mat):
        """Add add_mat to mat at (ind, ind)."""
        ix = np.ix_(ind, ind)
        if sparse.issparse(add_mat):
            add_mat = add_mat.toarray()
        mat[ix] += add_mat

    def funAdd2(mat, ind1, ind2, add_mat):
        """Add add_mat to mat at (ind1, ind2) and (ind2, ind1)."""
        ix1 = np.ix_(ind1, ind2)
        ix2 = np.ix_(ind2, ind1)
        if sparse.issparse(add_mat):
            add_mat = add_mat.toarray()
        mat[ix1] += add_mat
        mat[ix2] += add_mat

    def funDiag(size, value):
        return np.eye(size) * value

    # ---- Diagonal elements ----
    # Track (Rail)
    funAdd1(Mg, Model.Mesh.DOF.rail, Track.Rail.Mesh.Mg)
    funAdd1(Cg, Model.Mesh.DOF.rail, Track.Rail.Mesh.Cg)
    funAdd1(Kg, Model.Mesh.DOF.rail, Track.Rail.Mesh.Kg)

    # Pads to rail DOF
    n_s = Track.Sleeper.Tnum
    funAdd1(Cg, Model.Mesh.DOF.rail_vert_at_sleepers, funDiag(n_s, Track.Pad.Prop.c))
    funAdd1(Kg, Model.Mesh.DOF.rail_vert_at_sleepers, funDiag(n_s, Track.Pad.Prop.k))

    # Pads to sleepers DOF
    funAdd1(Cg, Model.Mesh.DOF.sleepers, funDiag(n_s, Track.Pad.Prop.c))
    funAdd1(Kg, Model.Mesh.DOF.sleepers, funDiag(n_s, Track.Pad.Prop.k))

    # Sleepers
    funAdd1(Mg, Model.Mesh.DOF.sleepers, funDiag(n_s, Track.Sleeper.Prop.m))

    # Ballast on approach to sleepers
    n_app = Track.Sleeper.num_app
    if n_app > 0:
        funAdd1(Cg, Model.Mesh.DOF.sleepers_app, funDiag(n_app, Track.Ballast.Prop.c))
        funAdd1(Kg, Model.Mesh.DOF.sleepers_app, funDiag(n_app, Track.Ballast.Prop.k))

    # Ballast on bridge to sleepers
    n_on = Track.Sleeper.num_onbeam
    funAdd1(Cg, Model.Mesh.DOF.sleepers_onbeam, funDiag(n_on, Track.BallastOnBeam.Prop.c))
    funAdd1(Kg, Model.Mesh.DOF.sleepers_onbeam, funDiag(n_on, Track.BallastOnBeam.Prop.k))

    # Ballast after bridge to sleepers
    n_aft = Track.Sleeper.num_aft
    if n_aft > 0:
        funAdd1(Cg, Model.Mesh.DOF.sleepers_aft, funDiag(n_aft, Track.Ballast.Prop.c))
        funAdd1(Kg, Model.Mesh.DOF.sleepers_aft, funDiag(n_aft, Track.Ballast.Prop.k))

    # Ballast on approach to Ballast DOF
    if n_app > 0:
        funAdd1(Cg, Model.Mesh.DOF.ballast_app, funDiag(n_app, Track.Ballast.Prop.c))
        funAdd1(Kg, Model.Mesh.DOF.ballast_app, funDiag(n_app, Track.Ballast.Prop.k))

    # Ballast on bridge to Bridge DOF
    funAdd1(Cg, Model.Mesh.DOF.beam_vert_under_sleeper, funDiag(n_on, Track.BallastOnBeam.Prop.c))
    funAdd1(Kg, Model.Mesh.DOF.beam_vert_under_sleeper, funDiag(n_on, Track.BallastOnBeam.Prop.k))

    # Ballast after bridge to Ballast DOF
    if n_aft > 0:
        funAdd1(Cg, Model.Mesh.DOF.ballast_aft, funDiag(n_aft, Track.Ballast.Prop.c))
        funAdd1(Kg, Model.Mesh.DOF.ballast_aft, funDiag(n_aft, Track.Ballast.Prop.k))

    # Ballast masses
    if n_app > 0:
        funAdd1(Mg, Model.Mesh.DOF.ballast_app, funDiag(n_app, Track.Ballast.Prop.m))
    # Ballast on bridge: distribute to all beam vert DOF
    funAdd1(Mg, Model.Mesh.DOF.beam_vert,
            funDiag(Beam.Mesh.Nodes.Tnum, Track.BallastOnBeam.Prop.m / Beam.Mesh.Ele.num_per_spacing))
    if n_aft > 0:
        funAdd1(Mg, Model.Mesh.DOF.ballast_aft, funDiag(n_aft, Track.Ballast.Prop.m))

    # Beam
    funAdd1(Mg, Model.Mesh.DOF.beam, Beam.Mesh.Mg)
    funAdd1(Cg, Model.Mesh.DOF.beam, Beam.Mesh.Cg)
    funAdd1(Kg, Model.Mesh.DOF.beam, Beam.Mesh.Kg)

    # Sub-Ballast
    if n_app > 0:
        funAdd1(Cg, Model.Mesh.DOF.ballast_app, funDiag(n_app, Track.SubBallast.Prop.c))
        funAdd1(Kg, Model.Mesh.DOF.ballast_app, funDiag(n_app, Track.SubBallast.Prop.k))
    if n_aft > 0:
        funAdd1(Cg, Model.Mesh.DOF.ballast_aft, funDiag(n_aft, Track.SubBallast.Prop.c))
        funAdd1(Kg, Model.Mesh.DOF.ballast_aft, funDiag(n_aft, Track.SubBallast.Prop.k))

    # ---- Off-Diagonal elements ----
    # Rail and Sleepers
    funAdd2(Cg, Model.Mesh.DOF.rail_vert_at_sleepers, Model.Mesh.DOF.sleepers,
            -funDiag(n_s, Track.Pad.Prop.c))
    funAdd2(Kg, Model.Mesh.DOF.rail_vert_at_sleepers, Model.Mesh.DOF.sleepers,
            -funDiag(n_s, Track.Pad.Prop.k))

    # Sleepers and Ballast on approach
    if n_app > 0:
        funAdd2(Cg, Model.Mesh.DOF.sleepers_app, Model.Mesh.DOF.ballast_app,
                -funDiag(n_app, Track.Ballast.Prop.c))
        funAdd2(Kg, Model.Mesh.DOF.sleepers_app, Model.Mesh.DOF.ballast_app,
                -funDiag(n_app, Track.Ballast.Prop.k))

    # Sleepers and Beam
    funAdd2(Cg, Model.Mesh.DOF.sleepers_onbeam, Model.Mesh.DOF.beam_vert_under_sleeper,
            -funDiag(n_on, Track.BallastOnBeam.Prop.c))
    funAdd2(Kg, Model.Mesh.DOF.sleepers_onbeam, Model.Mesh.DOF.beam_vert_under_sleeper,
            -funDiag(n_on, Track.BallastOnBeam.Prop.k))

    # Sleepers and Ballast after bridge
    if n_aft > 0:
        funAdd2(Cg, Model.Mesh.DOF.sleepers_aft, Model.Mesh.DOF.ballast_aft,
                -funDiag(n_aft, Track.Ballast.Prop.c))
        funAdd2(Kg, Model.Mesh.DOF.sleepers_aft, Model.Mesh.DOF.ballast_aft,
                -funDiag(n_aft, Track.Ballast.Prop.k))

    # Convert to CSC
    Model.Mesh.Mg = sparse.csc_matrix(Mg)
    Model.Mesh.Cg = sparse.csc_matrix(Cg)
    Model.Mesh.Kg = sparse.csc_matrix(Kg)

    # Symmetry check
    checksum = (
        sparse.linalg.norm(Model.Mesh.Mg - Model.Mesh.Mg.T) +
        sparse.linalg.norm(Model.Mesh.Cg - Model.Mesh.Cg.T) +
        sparse.linalg.norm(Model.Mesh.Kg - Model.Mesh.Kg.T)
    )
    if checksum > Calc.Cte.tol:
        raise ValueError('System matrices are not symmetric')

    # Auxiliary variables
    Model.Mesh.Ele.DOF = Track.Rail.Mesh.Ele.DOF
    Model.Mesh.Ele.a = Track.Rail.Mesh.Ele.a

    return Model
