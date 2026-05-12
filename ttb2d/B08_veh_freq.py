"""
B08 - Vehicle frequency calculation.
"""
import numpy as np


def B08_VehFreq(Veh_list, Calc):
    if getattr(Calc.Options, 'calc_veh_frq', 0) != 1:
        return Veh_list

    for veh in Veh_list:
        lam = np.real(np.linalg.eigvals(np.linalg.solve(veh.SysM.M, veh.SysM.K)))
        veh.Modal.w = np.sqrt(np.abs(lam))
        veh.Modal.f = veh.Modal.w / (2 * np.pi)

    return Veh_list
