"""
B10 - End Time calculation. Finds when the load reaches the end of the path.
"""
import numpy as np
import types


def B10_EndTime(Calc):
    if not hasattr(Calc, 'Time'):
        Calc.Time = types.SimpleNamespace()
    dt = 1.0
    L = Calc.Position.x_end - Calc.Position.x_0
    k_cont = True
    counter = 0
    max_counter = 100 + L / dt
    t1 = 0.0
    tol = Calc.Cte.tol

    while k_cont:
        t = np.array([t1, t1 + dt])
        counter += 1

        Lt = Calc.Position.v_0 * t + Calc.Position.a_0 * t ** Calc.Position.aa

        signs = np.sign(L - Lt)
        if np.sum(signs) == 0:
            if dt > tol:
                dt /= 2
            else:
                k_cont = False
        elif np.sum(signs) == 1:
            t = np.array([t[1], t[1]])
            k_cont = False
        else:
            t1 += dt

        if counter >= max_counter:
            raise ValueError('Initial position / Velocity / Acceleration are WRONG!')

    Calc.Time.t_end = np.mean(t)

    Calc.Position.v_end = (Calc.Position.v_0 +
                           Calc.Position.aa * Calc.Position.a_0 *
                           Calc.Time.t_end ** (Calc.Position.aa - 1))
    Calc.Position.a_end = (Calc.Position.aa *
                           (Calc.Position.aa - 1) * Calc.Position.a_0 *
                           Calc.Time.t_end ** (Calc.Position.aa - 2))

    Calc.Position.v_max = max(Calc.Position.v_0, Calc.Position.v_end)
    Calc.Position.v_min = min(Calc.Position.v_0, Calc.Position.v_end)
    Calc.Position.a_max = max(Calc.Position.a_0, Calc.Position.a_end)
    Calc.Position.a_min = min(Calc.Position.a_0, Calc.Position.a_end)

    return Calc
