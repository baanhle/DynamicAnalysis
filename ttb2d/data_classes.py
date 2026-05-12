"""
Data classes for TTB-2D simulation.
Uses nested SimpleNamespace for MATLAB-struct-like access (e.g. Beam.Prop.L).
"""
import types


def _ns(**kwargs):
    """Create a SimpleNamespace, optionally with initial values."""
    return types.SimpleNamespace(**kwargs)


def _has(obj, attr):
    """Check nested attribute existence, e.g. _has(obj, 'Prop.L')."""
    parts = attr.split('.')
    cur = obj
    for p in parts:
        if not hasattr(cur, p):
            return False
        cur = getattr(cur, p)
    return True


def _get(obj, attr, default=None):
    """Get nested attribute with default."""
    parts = attr.split('.')
    cur = obj
    for p in parts:
        if not hasattr(cur, p):
            return default
        cur = getattr(cur, p)
    return cur


class Beam:
    def __init__(self):
        self.Prop = _ns()
        self.BC = _ns()
        self.Mesh = _ns(Ele=_ns(), Nodes=_ns(), DOF=_ns())
        self.Options = _ns()
        self.Damping = _ns(per=0)
        self.Modal = _ns()


class Track:
    def __init__(self):
        self.Rail = _ns(Prop=_ns(), Mesh=_ns(Ele=_ns(), Nodes=_ns(), DOF=_ns()),
                        Options=_ns(), Damping=_ns(per=0), BC=_ns(), Modal=_ns())
        self.Pad = _ns(Prop=_ns())
        self.Sleeper = _ns(Prop=_ns())
        self.Ballast = _ns(Prop=_ns())
        self.SubBallast = _ns(Prop=_ns())
        self.BallastOnBeam = _ns(included=0, Prop=_ns())
        self.PadUnderSleeperOnBeam = _ns(included=0, Prop=_ns())
        self.Load = _ns()


class Veh:
    def __init__(self):
        self.Body = _ns()
        self.Bogie = _ns()
        self.Wheels = _ns()
        self.Susp = _ns(Prim=_ns(), Sec=_ns())
        self.SysM = _ns()
        self.Modal = _ns()
        self.DOF = _ns()


class Train:
    def __init__(self):
        self.vel = 0
        self.Veh = []  # list of Veh
        self.Load = _ns(path='', file='')


class Calc:
    def __init__(self):
        self.Options = _ns()
        self.Profile = _ns()
        self.Position = _ns()
        self.Solver = _ns()
        self.Time = _ns()
        self.Plot = _ns(Veh=_ns(), Model=_ns(), Beam=_ns())
        self.Cte = _ns(tol=1e-6, grav=-9.81)
        self.Veh = []  # list of per-vehicle calc data
        self.Train = _ns()
        self.Type = _ns(short_text='COUP')


class Model:
    def __init__(self):
        self.Mesh = _ns(DOF=_ns(), Ele=_ns(), XLoc=_ns())
        self.BC = _ns()
        self.Modal = _ns()


class Sol:
    def __init__(self):
        self.Veh = []
        self.Model = _ns(Nodal=_ns())
        self.Beam = _ns()
        self.contactLost = 0
