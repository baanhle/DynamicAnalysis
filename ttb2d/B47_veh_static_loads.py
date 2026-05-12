"""
B47 - Vehicle Static Loads
Calculates the static loads of the vehicles using their system matrices.
"""
import numpy as np


def B47_VehStaticLoads(Veh_list, Calc):
    num_veh = Veh_list[0].Tnum if hasattr(Veh_list[0], 'Tnum') else len(Veh_list)

    for veh_num in range(num_veh):
        veh = Veh_list[veh_num]

        Fext = veh.SysM.M @ (veh.DOF.vert * Calc.Cte.grav)
        veh.U0 = np.linalg.solve(veh.SysM.K, Fext)
        wheel_disp = veh.Wheels.N2w @ veh.U0
        veh.sta_loads = veh.Susp.Prim.k * wheel_disp + veh.Wheels.m * Calc.Cte.grav

        # Check
        total_mass = veh.Body.m + np.sum(veh.Bogie.m) + np.sum(veh.Wheels.m)
        check = np.sum(veh.sta_loads) - total_mass * Calc.Cte.grav
        if abs(check) > Calc.Cte.tol:
            print(f'Static weight of vehicle {veh_num + 1} is not correct')

    return Veh_list
