"""
B18 - Vehicle System Mass, Damping and Stiffness Matrices.
6-DOF vehicle model: [body_vert, bogie1_vert, bogie2_vert, body_rot, bogie1_rot, bogie2_rot]
Faithfully translated from MATLAB B18_TrainVehEq.m.
"""
import numpy as np
import types


def B18_TrainVehEq(Veh):
    """
    Build 6x6 vehicle system matrices (M, K, C) and set N2w, ktn, ctn, mtn.
    """
    m   = Veh.Body.m
    I   = Veh.Body.I
    mB1 = float(np.atleast_1d(Veh.Bogie.m)[0])
    mB2 = float(np.atleast_1d(Veh.Bogie.m)[1]) if len(np.atleast_1d(Veh.Bogie.m)) > 1 else mB1
    IB1 = float(np.atleast_1d(Veh.Bogie.I)[0])
    IB2 = float(np.atleast_1d(Veh.Bogie.I)[1]) if len(np.atleast_1d(Veh.Bogie.I)) > 1 else IB1

    k1 = float(np.atleast_1d(Veh.Susp.Prim.k)[0])
    k2 = float(np.atleast_1d(Veh.Susp.Prim.k)[1]) if len(np.atleast_1d(Veh.Susp.Prim.k)) > 1 else k1
    k3 = float(np.atleast_1d(Veh.Susp.Prim.k)[2]) if len(np.atleast_1d(Veh.Susp.Prim.k)) > 2 else k1
    k4 = float(np.atleast_1d(Veh.Susp.Prim.k)[3]) if len(np.atleast_1d(Veh.Susp.Prim.k)) > 3 else k2
    c1 = float(np.atleast_1d(Veh.Susp.Prim.c)[0])
    c2 = float(np.atleast_1d(Veh.Susp.Prim.c)[1]) if len(np.atleast_1d(Veh.Susp.Prim.c)) > 1 else c1
    c3 = float(np.atleast_1d(Veh.Susp.Prim.c)[2]) if len(np.atleast_1d(Veh.Susp.Prim.c)) > 2 else c1
    c4 = float(np.atleast_1d(Veh.Susp.Prim.c)[3]) if len(np.atleast_1d(Veh.Susp.Prim.c)) > 3 else c2
    ks1 = float(np.atleast_1d(Veh.Susp.Sec.k)[0])
    ks2 = float(np.atleast_1d(Veh.Susp.Sec.k)[1]) if len(np.atleast_1d(Veh.Susp.Sec.k)) > 1 else ks1
    cs1 = float(np.atleast_1d(Veh.Susp.Sec.c)[0])
    cs2 = float(np.atleast_1d(Veh.Susp.Sec.c)[1]) if len(np.atleast_1d(Veh.Susp.Sec.c)) > 1 else cs1

    bl = np.atleast_1d(Veh.Bogie.L)
    dB1_F = float(bl[0]) / 2.0
    dB1_B = float(bl[0]) / 2.0
    dB2_F = float(bl[1]) / 2.0 if len(bl) > 1 else dB1_F
    dB2_B = float(bl[1]) / 2.0 if len(bl) > 1 else dB1_B

    # Body reference distances
    if hasattr(Veh.Body, 'L_F') and Veh.Body.L_F is not None:
        d_F = float(Veh.Body.L_F)
        d_B = float(Veh.Body.L_B)
    else:
        d_F = float(Veh.Body.L) / 2.0
        d_B = float(Veh.Body.L) / 2.0

    # ---- Mass matrix ----
    M = np.diag([m, mB1, mB2, I, IB1, IB2])

    # ---- Stiffness matrix (exact match with MATLAB) ----
    K = np.zeros((6, 6))
    K[0, 0] = ks1 + ks2
    K[0, 1] = -ks1;   K[1, 0] = -ks1
    K[0, 2] = -ks2;   K[2, 0] = -ks2
    K[0, 3] = d_F*ks1 - d_B*ks2;   K[3, 0] = d_F*ks1 - d_B*ks2

    K[1, 1] = k1 + k2 + ks1
    K[1, 3] = -d_F*ks1;   K[3, 1] = -d_F*ks1
    K[1, 4] = dB1_F*k1 - dB1_B*k2;   K[4, 1] = dB1_F*k1 - dB1_B*k2

    K[2, 2] = k3 + k4 + ks2
    K[2, 3] = d_B*ks2;    K[3, 2] = d_B*ks2
    K[2, 5] = dB2_F*k3 - dB2_B*k4;   K[5, 2] = dB2_F*k3 - dB2_B*k4

    K[3, 3] = d_F**2 * ks1 + d_B**2 * ks2
    K[4, 4] = k1*dB1_F**2 + k2*dB1_B**2
    K[5, 5] = k3*dB2_F**2 + k4*dB2_B**2

    # ---- Damping matrix (same structure as K with c instead of k) ----
    C = np.zeros((6, 6))
    C[0, 0] = cs1 + cs2
    C[0, 1] = -cs1;   C[1, 0] = -cs1
    C[0, 2] = -cs2;   C[2, 0] = -cs2
    C[0, 3] = cs1*d_F - cs2*d_B;   C[3, 0] = cs1*d_F - cs2*d_B

    C[1, 1] = c1 + c2 + cs1
    C[1, 3] = -cs1*d_F;   C[3, 1] = -cs1*d_F
    C[1, 4] = dB1_F*c1 - dB1_B*c2;   C[4, 1] = dB1_F*c1 - dB1_B*c2

    C[2, 2] = c3 + c4 + cs2
    C[2, 3] = cs2*d_B;    C[3, 2] = cs2*d_B
    C[2, 5] = dB2_F*c3 - dB2_B*c4;   C[5, 2] = dB2_F*c3 - dB2_B*c4

    C[3, 3] = cs1*d_F**2 + cs2*d_B**2
    C[4, 4] = c1*dB1_F**2 + c2*dB1_B**2
    C[5, 5] = c3*dB2_F**2 + c4*dB2_B**2

    # ---- N2w: nodal displacements → wheel displacements ----
    # MATLAB: wheel1=[0,1,0,0,+dB1_F,0], wheel2=[0,1,0,0,-dB1_B,0]
    #         wheel3=[0,0,1,0,0,+dB2_F], wheel4=[0,0,1,0,0,-dB2_B]
    Veh.Wheels.N2w = np.array([
        [0, 1, 0, 0,  dB1_F,      0],   # wheel 1: front of bogie1
        [0, 1, 0, 0, -dB1_B,      0],   # wheel 2: rear of bogie1
        [0, 0, 1, 0,  0,       dB2_F],  # wheel 3: front of bogie2
        [0, 0, 1, 0,  0,      -dB2_B],  # wheel 4: rear of bogie2
    ], dtype=float)

    Veh.SysM = types.SimpleNamespace()
    Veh.SysM.M = M
    Veh.SysM.K = K
    Veh.SysM.C = C

    Veh.Tnum_DOF = 6
    Veh.ktn = np.array([k1, k2, k3, k4])
    Veh.ctn = np.array([c1, c2, c3, c4])
    Veh.mtn = np.atleast_1d(Veh.Wheels.m).astype(float)

    Veh.DOF = types.SimpleNamespace()
    Veh.DOF.vert = np.array([1, 1, 1, 0, 0, 0], dtype=float)
    Veh.DOF.rot  = np.array([0, 0, 0, 1, 1, 1], dtype=float)

    return Veh

    p = Veh.Prop

    mb = p.mb; Ib = p.Ib
    mt = p.mt; It = p.It

    kp = p.kp; cp = p.cp
    ks = p.ks; cs = p.cs

    Ls = p.Ls; Lt = p.Lt

    # Mass matrix
    M = np.diag([mb, mt, mt, Ib, It, It])

    # Stiffness matrix
    K = np.zeros((6, 6))
    K[0, 0] = 2 * ks
    K[0, 1] = -ks
    K[0, 2] = -ks
    K[1, 0] = -ks
    K[1, 1] = ks + 2 * kp
    K[2, 0] = -ks
    K[2, 2] = ks + 2 * kp

    K[3, 3] = 2 * ks * Ls ** 2
    K[3, 4] = -ks * Ls
    K[3, 5] = ks * Ls
    K[4, 3] = -ks * Ls
    K[4, 4] = ks * Ls + 2 * kp * Lt ** 2
    K[5, 3] = ks * Ls
    K[5, 5] = ks * Ls + 2 * kp * Lt ** 2

    # Coupling between vert and rot
    K[0, 4] = -ks * Ls
    K[0, 5] = ks * Ls
    K[1, 3] = -ks * Ls
    K[2, 3] = ks * Ls
    K[4, 0] = -ks * Ls
    K[5, 0] = ks * Ls
    K[3, 1] = -ks * Ls
    K[3, 2] = ks * Ls

    K[1, 4] = ks * Ls
    K[4, 1] = ks * Ls
    K[2, 5] = -ks * Ls
    K[5, 2] = -ks * Ls

    # Damping matrix
    C = np.zeros((6, 6))
    C[0, 0] = 2 * cs
    C[0, 1] = -cs
    C[0, 2] = -cs
    C[1, 0] = -cs
    C[1, 1] = cs + 2 * cp
    C[2, 0] = -cs
    C[2, 2] = cs + 2 * cp

    C[3, 3] = 2 * cs * Ls ** 2
    C[3, 4] = -cs * Ls
    C[3, 5] = cs * Ls
    C[4, 3] = -cs * Ls
    C[4, 4] = cs * Ls + 2 * cp * Lt ** 2
    C[5, 3] = cs * Ls
    C[5, 5] = cs * Ls + 2 * cp * Lt ** 2

    C[0, 4] = -cs * Ls
    C[0, 5] = cs * Ls
    C[1, 3] = -cs * Ls
    C[2, 3] = cs * Ls
    C[4, 0] = -cs * Ls
    C[5, 0] = cs * Ls
    C[3, 1] = -cs * Ls
    C[3, 2] = cs * Ls

    C[1, 4] = cs * Ls
    C[4, 1] = cs * Ls
    C[2, 5] = -cs * Ls
    C[5, 2] = -cs * Ls

    Veh.SysM = types.SimpleNamespace()
    Veh.SysM.M = M
    Veh.SysM.K = K
    Veh.SysM.C = C

    return Veh
