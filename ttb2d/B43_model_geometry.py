"""
B43 - Model Geometry
Calculates geometric properties, position variables, and profile lengths.
"""
import numpy as np
import types
import math


def B43_ModelGeometry(Calc, Train, Track, Beam):
    # Model redux option
    if not hasattr(Calc, 'Options'):
        Calc.Options = types.SimpleNamespace()
    if not hasattr(Calc.Options, 'redux'):
        Calc.Options.redux = 1
    Calc.Options.redux_factor = int(Calc.Options.redux == 0)

    if not hasattr(Calc, 'Profile'):
        Calc.Profile = types.SimpleNamespace()

    # Approach distance
    Calc.Profile.L_Approach = (
        round(Calc.Profile.minL_Approach / Track.Sleeper.spacing) * Track.Sleeper.spacing
    )

    # After crossing
    if not hasattr(Calc.Profile, 'minL_After'):
        Calc.Profile.minL_After = 10
    Calc.Profile.L_After = (
        round(Calc.Profile.minL_After / Track.Sleeper.spacing) * Track.Sleeper.spacing
    )

    # Beam elements
    Beam.Mesh.Ele.L = Track.Sleeper.spacing / Beam.Mesh.Ele.num_per_spacing
    Beam.Mesh.Ele.num = round(Beam.Prop.L / Beam.Mesh.Ele.L)
    Beam.Prop.L = Beam.Mesh.Ele.L * Beam.Mesh.Ele.num
    Calc.Profile.L_bridge = Beam.Prop.L
    Beam.Prop.A = 1

    # Vehicle variables
    Veh = Train.Veh.data
    num_veh = len(Veh)
    Veh[0].Tnum = num_veh

    # Total train length
    Veh[0].TL = Veh[0].Body.L + Veh[0].Bogie.L[0] / 2
    Veh[0].L_app = Calc.Profile.L_Approach

    for v in range(1, num_veh):
        Veh[0].TL += Veh[v - 1].Body.Le[1] + Veh[v].Body.Le[0] + Veh[v].Body.L
        Veh[v].L_app = (Veh[v - 1].L_app +
                        Veh[v - 1].Bogie.L[0] / 2 + Veh[v - 1].Body.L +
                        Veh[v - 1].Body.Le[1] + Veh[v].Body.Le[0] -
                        Veh[v].Bogie.L[0] / 2)

    Veh[0].TL += Veh[-1].Bogie.L[1] / 2

    # Geometric properties for each vehicle
    if not hasattr(Calc, 'Train'):
        Calc.Train = types.SimpleNamespace()
    if not hasattr(Calc.Train, 'Veh'):
        Calc.Train.Veh = [types.SimpleNamespace() for _ in range(num_veh)]

    for v in range(num_veh):
        Veh[v].Ax_dist = np.array([
            0,
            Veh[v].Bogie.L[0],
            Veh[v].Bogie.L[0] / 2 + Veh[v].Body.L - Veh[v].Bogie.L[1] / 2,
            Veh[v].Bogie.L[0] / 2 + Veh[v].Body.L + Veh[v].Bogie.L[1] / 2
        ])
        Veh[v].wheelbase = Veh[v].Ax_dist[-1]
        Veh[v].First_wheel_dist = Veh[v].L_app - Veh[0].L_app
        Calc.Train.Veh[v].Positive_ContactForce = 0

    # Vehicle total length rounded up
    Veh[0].max_TL = math.ceil(Veh[0].TL / Track.Sleeper.spacing) * Track.Sleeper.spacing
    Calc.Profile.max_TL = Veh[0].max_TL

    # Additional model length
    Calc.Profile.extra_L = Beam.Prop.L % Track.Sleeper.spacing
    if Calc.Profile.extra_L > 0:
        Calc.Profile.extra_L = Track.Sleeper.spacing - Calc.Profile.extra_L

    # Additional sleepers
    Track.Rail.Options = types.SimpleNamespace() if not hasattr(Track.Rail, 'Options') else Track.Rail.Options
    Track.Rail.Options.num_add_sleepers = 10
    if Calc.Options.redux == 1:
        Track.Rail.Options.num_add_sleepers = 0

    Calc.Profile.extra_L2 = Track.Rail.Options.num_add_sleepers * Track.Sleeper.spacing

    # Minimum total profile length
    rf = Calc.Options.redux_factor
    Calc.Profile.L = (
        2 * Calc.Profile.max_TL * rf +
        Veh[0].L_app + Beam.Prop.L + Calc.Profile.extra_L +
        2 * Calc.Profile.extra_L2 * rf + Calc.Profile.L_After
    )
    Calc.Profile.L = round(Calc.Profile.L / Track.Sleeper.spacing) * Track.Sleeper.spacing

    # Vehicle's initial position
    if not hasattr(Calc, 'Position'):
        Calc.Position = types.SimpleNamespace()
    Calc.Position.x_start_end = np.array([
        (Veh[0].max_TL + Calc.Profile.extra_L2) * rf,
        Calc.Profile.L - Calc.Profile.extra_L2 * rf +
        Veh[0].max_TL * (1 - rf)
    ])

    Calc.Profile.L_Aw = Calc.Profile.L_Approach + Calc.Profile.max_TL * rf

    return Calc, Train, Beam
