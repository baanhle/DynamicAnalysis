"""
B07 - Options Processing
Processes input variables, sets defaults, and generates auxiliary variables.
"""
import types
import numpy as np


def _setdefault(ns, attr, value):
    """Set attribute on SimpleNamespace if not already present."""
    if not hasattr(ns, attr):
        setattr(ns, attr, value)


def B07_OptionsProcessing(Calc, Train, Track, Beam):
    # Constants
    if not hasattr(Calc, 'Cte'):
        Calc.Cte = types.SimpleNamespace()
    Calc.Cte.tol = 1e-6
    Calc.Cte.grav = -9.81

    # Type of calculation
    if not hasattr(Calc, 'Type'):
        Calc.Type = types.SimpleNamespace()
        Calc.Type.short_text = 'COUP'
    if Calc.Type.short_text == 'COUP':
        Calc.Type.id = 1
        Calc.Type.long_text = 'Coupled system calculation'

    # Options
    if not hasattr(Calc, 'Options'):
        Calc.Options = types.SimpleNamespace()
    _setdefault(Calc.Options, 'calc_beam_sections', np.array([]))
    if len(np.atleast_1d(Calc.Options.calc_beam_sections)) == 0:
        Calc.Options.num_calc_beam_sections = 0
    else:
        Calc.Options.num_calc_beam_sections = len(Calc.Options.calc_beam_sections)
    _setdefault(Calc.Options, 'calc_model_frq', 0)
    _setdefault(Calc.Options, 'calc_model_modes', 0)
    _setdefault(Calc.Options, 'VBI', 1)
    _setdefault(Calc.Options, 'disp_every', 2)

    # Solver
    if not hasattr(Calc, 'Solver'):
        Calc.Solver = types.SimpleNamespace()
        Calc.Solver.was_empty = 1
    _setdefault(Calc.Solver, 'NewMark_damp', 0)
    if Calc.Solver.NewMark_damp == 0:
        Calc.Solver.NewMark_delta = 0.5
        Calc.Solver.NewMark_beta = 0.25
    elif Calc.Solver.NewMark_damp == 1:
        Calc.Solver.NewMark_delta = 0.6
        Calc.Solver.NewMark_beta = 0.3025

    # Profile
    if not hasattr(Calc, 'Profile'):
        Calc.Profile = types.SimpleNamespace()
    _setdefault(Calc.Profile, 'Type', 0)
    ax_dists = []
    for veh in Train.Veh.data:
        ax_dists.extend(list(np.diff(veh.Ax_dist)))
    _setdefault(Calc.Profile, 'min_dx', min(np.abs(ax_dists)) if ax_dists else 0.01)

    # Plotting defaults
    if not hasattr(Calc, 'Plot'):
        Calc.Plot = types.SimpleNamespace()
        Calc.Plot.NoPlot = 1
    if not hasattr(Calc.Plot, 'Veh'):
        Calc.Plot.Veh = types.SimpleNamespace()
        Calc.Plot.Veh.NoPlot = 1
    if not hasattr(Calc.Plot, 'Model'):
        Calc.Plot.Model = types.SimpleNamespace()
        Calc.Plot.Model.NoPlot = 1
    if not hasattr(Calc.Plot, 'Beam'):
        Calc.Plot.Beam = types.SimpleNamespace()
        Calc.Plot.Beam.NoPlot = 1

    _setdefault(Calc.Plot, 'P1_Beam_frq', 0)
    _setdefault(Calc.Plot, 'P2_Beam_modes', 0)
    _setdefault(Calc.Plot, 'P3_VehPos', 0)
    _setdefault(Calc.Plot, 'Profile_original', 0)
    _setdefault(Calc.Plot, 'Model_modes', [])

    if hasattr(Calc.Plot, 'P1_Beam_frq') and Calc.Plot.P1_Beam_frq == 1:
        Calc.Options.calc_beam_frq = 1
    if hasattr(Calc.Plot, 'P2_Beam_modes') and Calc.Plot.P2_Beam_modes >= 1:
        Calc.Options.calc_beam_frq = 1
        Calc.Options.calc_beam_modes = 1

    # Vehicle plot defaults
    for attr in ['P01_VertDisp', 'P02_VertVel', 'P03_VertAcc',
                 'P04_ContactForce_t', 'P05_ContactForce_x']:
        _setdefault(Calc.Plot.Veh, attr, 0)
    # Model plot defaults
    _setdefault(Calc.Plot.Model, 'P00_ModelVisualization', 0)
    _setdefault(Calc.Plot.Model, 'P01_ModelDef', -1)
    _setdefault(Calc.Plot.Model, 'P02_ModelRot', -1)
    # Beam plot defaults
    for attr in ['P01_DispContour', 'P02_StaticDispContour', 'P03_BMContour',
                 'P04_StaticBMContour', 'P05_ShearContour', 'P06_StaticShearContour',
                 'P07_VertAccContour', 'P08_Sections_BeamVertDisp',
                 'P09_Sections_BeamBM', 'P10_Sections_BeamShear', 'P11_Sections_BeamAcc']:
        _setdefault(Calc.Plot.Beam, attr, 0)

    # Vehicle
    Train.Veh.data[0].VelAcc = [Train.vel, 0, 2]
    _setdefault(Calc.Options, 'calc_veh_frq', 1)

    Calc.Position = types.SimpleNamespace() if not hasattr(Calc, 'Position') else Calc.Position
    Calc.Position.x_0 = Calc.Position.x_start_end[0]
    Calc.Position.x_end = Calc.Position.x_start_end[1]
    Calc.Position.v_0 = Train.Veh.data[0].VelAcc[0]
    Calc.Position.a_0 = Train.Veh.data[0].VelAcc[1]
    Calc.Position.aa = Train.Veh.data[0].VelAcc[2]

    # Beam
    if not hasattr(Beam, 'Options'):
        Beam.Options = types.SimpleNamespace()
    Beam.Options.k_Mconsist = 1

    # BC text processing
    if Beam.BC.text == 'SP':
        Beam.BC.loc = np.array([0, Beam.Prop.L])
        Beam.BC.vert_stiff = np.array([-1, -1])
        Beam.BC.rot_stiff = np.array([0, 0])
        Beam.BC.text_long = 'Simply Supported'
    elif Beam.BC.text == 'FF':
        Beam.BC.loc = np.array([0, Beam.Prop.L])
        Beam.BC.vert_stiff = np.array([-1, -1])
        Beam.BC.rot_stiff = np.array([-1, -1])
        Beam.BC.text_long = 'Fixed-Fixed'

    _setdefault(Calc.Options, 'calc_beam_frq', 1)
    if not hasattr(Calc.Options, 'calc_beam_modes'):
        Calc.Options.calc_beam_frq = 1
        Calc.Options.calc_beam_modes = 1

    if not hasattr(Beam, 'Damping'):
        Beam.Damping = types.SimpleNamespace()
        Beam.Damping.per = 0

    _setdefault(Calc.Options, 'BM_calc_mode', 1)
    _setdefault(Calc.Options, 'Shear_calc_mode', 1)

    # Track
    if not hasattr(Track.Rail, 'Damping'):
        Track.Rail.Damping = types.SimpleNamespace()
        Track.Rail.Damping.per = 0

    # Checks
    if Calc.Options.num_calc_beam_sections > 0:
        secs = np.atleast_1d(Calc.Options.calc_beam_sections)
        if np.any(secs < 0) or np.any(secs > Beam.Prop.L):
            raise ValueError('Beam calculation section not on beam')

    if not hasattr(Track, 'BallastOnBeam'):
        Track.BallastOnBeam = types.SimpleNamespace()
        Track.BallastOnBeam.included = 0
    else:
        _setdefault(Track.BallastOnBeam, 'included', 1)

    if not hasattr(Track, 'PadUnderSleeperOnBeam'):
        Track.PadUnderSleeperOnBeam = types.SimpleNamespace()
        Track.PadUnderSleeperOnBeam.included = 0
    else:
        _setdefault(Track.PadUnderSleeperOnBeam, 'included', 1)

    check = Track.BallastOnBeam.included + Track.PadUnderSleeperOnBeam.included
    if check == 0:
        raise ValueError('Missing information for "Ballast on Beam" or "Pad under Sleeper on Beam"')
    elif check == 2:
        raise ValueError('Properties defined simultaneously for "Ballast on Beam" and "Pad under Sleeper on Beam"')

    spacing = float(Track.Sleeper.spacing)
    rem = float(Calc.Profile.L % spacing)
    rem_to_grid = min(abs(rem), abs(spacing - rem))
    if rem_to_grid > Calc.Cte.tol:
        raise ValueError('Profile length is wrong')

    if hasattr(Calc.Options, 'redux') and Calc.Options.redux == 0:
        print('No redux model is used! Are you sure about this?')

    # Beam frequency factor defaults
    _setdefault(Calc.Options, 'beam_frq_factor', 20)
    _setdefault(Calc.Options, 'veh_frq_factor', 20)
    _setdefault(Calc.Options, 'min_Nele', 4)

    # Max velocity (for dt from space)
    Calc.Position.v_max = max(abs(Calc.Position.v_0), 1.0)

    return Calc, Train, Track, Beam
