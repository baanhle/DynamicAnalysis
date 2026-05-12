"""
B11 - Time and Space Discretization.
"""
import numpy as np
import types


def B11_TimeSpaceDiscretization(Calc, Beam, Train, Track):
    # -- Time step --
    min_beam_frq = Beam.Modal.w[Beam.Modal.num_rigid_modes] / (2 * np.pi)

    dt_from_beam = 1.0 / (Calc.Options.beam_frq_factor * min_beam_frq)

    # Vehicle frequencies (if available)
    has_veh_frq = False
    for tr in range(Train.Veh.num):
        if hasattr(Train.Veh.data[tr], 'Modal') and hasattr(Train.Veh.data[tr].Modal, 'f'):
            has_veh_frq = True

    if has_veh_frq:
        veh_max_f = 0
        for tr in range(Train.Veh.num):
            if hasattr(Train.Veh.data[tr], 'Modal') and hasattr(Train.Veh.data[tr].Modal, 'f'):
                veh_max_f = max(veh_max_f, np.max(Train.Veh.data[tr].Modal.f))
        dt_from_veh = 1.0 / (Calc.Options.veh_frq_factor * veh_max_f) if veh_max_f > 0 else dt_from_beam
    else:
        dt_from_veh = dt_from_beam

    # Smallest element length
    min_ele = np.min(Beam.Mesh.Ele.a)
    dt_from_space = min_ele / (Calc.Position.v_max * Calc.Options.min_Nele)

    dt = min(dt_from_beam, dt_from_space, dt_from_veh)
    Calc.Solver.dt = dt

    # -- Fix for track spacing (if coupled) --
    if hasattr(Calc.Options, 'model_type'):
        # nothing special needed for basic case
        pass

    # -- Time array --
    Calc.Solver.t0_ind = 0
    t = np.arange(0.0, Calc.Time.t_end + dt / 2, dt)
    Calc.Solver.t = t
    Calc.Solver.num_t = len(t)

    # -- Position array --
    Calc.Position.x = (Calc.Position.x_0 +
                       Calc.Position.v_0 * t +
                       Calc.Position.a_0 * t ** Calc.Position.aa)

    return Calc, Beam, Train, Track
