"""
Track property definitions.
Each function returns a Track SimpleNamespace object.
"""
import types
import numpy as np


def _ns():
    return types.SimpleNamespace()


def TrackProp_Zhai_WithBallastOnBridge():
    """
    Zhai et al. track properties WITH Ballast on Bridge.
    Source: Zhai, Wang, Lin (2004) J. Sound Vib. 270(4-5), 673-683.
    """
    Track = _ns()

    # Rail
    Track.Rail = _ns()
    Track.Rail.Prop = _ns()
    Track.Rail.Prop.E = 2.059e11
    Track.Rail.Prop.I = 3.217e-5 * 2
    Track.Rail.Prop.rho = 60.64 * 2
    Track.Rail.Damping = _ns()
    Track.Rail.Damping.per = 0.1
    Track.Rail.Mesh = _ns()
    Track.Rail.Mesh.Ele = _ns()

    # Pad
    Track.Pad = _ns()
    Track.Pad.Prop = _ns()
    Track.Pad.Prop.k = 6.5e7
    Track.Pad.Prop.c = 7.5e4

    # Sleeper
    Track.Sleeper = _ns()
    Track.Sleeper.spacing = 0.6
    Track.Sleeper.Prop = _ns()
    Track.Sleeper.Prop.m = 125.5 * 2

    # Ballast
    Track.Ballast = _ns()
    Track.Ballast.Prop = _ns()
    Track.Ballast.Prop.m = 531.4
    Track.Ballast.Prop.k = 137.75e6
    Track.Ballast.Prop.c = 5.88e4

    # Sub-Ballast
    Track.SubBallast = _ns()
    Track.SubBallast.Prop = _ns()
    Track.SubBallast.Prop.k = 77.5e6
    Track.SubBallast.Prop.c = 3.115e4

    # Ballast on Bridge
    Track.BallastOnBeam = _ns()
    Track.BallastOnBeam.Prop = _ns()
    Track.BallastOnBeam.Prop.m = Track.Ballast.Prop.m
    Track.BallastOnBeam.Prop.k = Track.Ballast.Prop.k
    Track.BallastOnBeam.Prop.c = Track.Ballast.Prop.c
    Track.BallastOnBeam.included = 1

    # PadUnderSleeperOnBeam not included
    Track.PadUnderSleeperOnBeam = _ns()
    Track.PadUnderSleeperOnBeam.included = 0

    return Track


def TrackProp_Zhai_NoBallastOnBridge():
    """
    Zhai et al. track properties WITHOUT Ballast on Bridge (uses pad under sleeper).
    """
    Track = _ns()

    # Rail
    Track.Rail = _ns()
    Track.Rail.Prop = _ns()
    Track.Rail.Prop.E = 2.059e11
    Track.Rail.Prop.I = 3.217e-5 * 2
    Track.Rail.Prop.rho = 60.64 * 2
    Track.Rail.Damping = _ns()
    Track.Rail.Damping.per = 0.1
    Track.Rail.Mesh = _ns()
    Track.Rail.Mesh.Ele = _ns()

    # Pad
    Track.Pad = _ns()
    Track.Pad.Prop = _ns()
    Track.Pad.Prop.k = 6.5e7
    Track.Pad.Prop.c = 7.5e4

    # Sleeper
    Track.Sleeper = _ns()
    Track.Sleeper.spacing = 0.6
    Track.Sleeper.Prop = _ns()
    Track.Sleeper.Prop.m = 125.5 * 2

    # Ballast
    Track.Ballast = _ns()
    Track.Ballast.Prop = _ns()
    Track.Ballast.Prop.m = 531.4
    Track.Ballast.Prop.k = 137.75e6
    Track.Ballast.Prop.c = 5.88e4

    # Sub-Ballast
    Track.SubBallast = _ns()
    Track.SubBallast.Prop = _ns()
    Track.SubBallast.Prop.k = 77.5e6
    Track.SubBallast.Prop.c = 3.115e4

    # No BallastOnBeam
    Track.BallastOnBeam = _ns()
    Track.BallastOnBeam.included = 0
    # Keep zeroed properties so downstream solver code can safely access
    # Track.BallastOnBeam.Prop.* even when this layer is disabled.
    Track.BallastOnBeam.Prop = _ns()
    Track.BallastOnBeam.Prop.m = 0.0
    Track.BallastOnBeam.Prop.k = 0.0
    Track.BallastOnBeam.Prop.c = 0.0

    # Pad under sleeper on bridge
    Track.PadUnderSleeperOnBeam = _ns()
    Track.PadUnderSleeperOnBeam.included = 1
    Track.PadUnderSleeperOnBeam.Prop = _ns()
    Track.PadUnderSleeperOnBeam.Prop.k = 120e6
    Track.PadUnderSleeperOnBeam.Prop.c = 60e4

    return Track
