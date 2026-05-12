"""
B65 - Dynamic Calculation (Coupled System) - Faster version
Main Newmark-Beta time integration solver for the coupled vehicle-track-bridge system.
Uses sparse matrix assembly for time-dependent coupling terms.
"""
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import spsolve
import time as time_module
from .B03_beam_matrices import shape_fun, shape_fun_p, shape_fun_pp


def B65_DynamicCalcCoupledFaster(Veh_list, Model, Calc, Track, Sol):
    num_veh = Veh_list[0].Tnum if hasattr(Veh_list[0], 'Tnum') else len(Veh_list)
    global_ind_end = int(Veh_list[-1].global_ind[-1]) + 1
    Coup_DOF_Tnum = global_ind_end + Model.Mesh.DOF.Tnum
    num_t = Calc.Solver.num_t

    # Initialize solution arrays
    Coup_U = np.zeros((Coup_DOF_Tnum, num_t))
    Coup_V = np.zeros((Coup_DOF_Tnum, num_t))
    Coup_A = np.zeros((Coup_DOF_Tnum, num_t))

    # Coupled BC
    BC_DOF_fixed = (global_ind_end + Model.BC.DOF_fixed).astype(int)
    num_DOF_fixed = len(BC_DOF_fixed)

    # Auxiliary variables
    vel = Veh_list[0].vel
    vel2 = vel ** 2
    ele_DOF = Track.Rail.Mesh.Ele.DOF
    grav = Calc.Cte.grav
    dt = Calc.Solver.dt
    beta = Calc.Solver.NewMark_beta
    delta = Calc.Solver.NewMark_delta

    NB = np.zeros(6)
    NB[0] = 1.0 / (beta * dt ** 2)
    NB[1] = delta / (beta * dt)
    NB[2] = 1.0 / (beta * dt)
    NB[3] = 1.0 / (2 * beta) - 1.0
    NB[4] = 1.0 - delta / beta
    NB[5] = (1.0 - delta / (2 * beta)) * dt

    # Initial conditions
    for veh_num in range(num_veh):
        gi = Veh_list[veh_num].global_ind
        Coup_U[gi, 0] = Sol.Veh[veh_num].U0
        Coup_V[gi, 0] = Sol.Veh[veh_num].V0
        Coup_A[gi, 0] = Sol.Veh[veh_num].A0
    Coup_U[global_ind_end:, 0] = Sol.Model.Nodal.U0
    Coup_V[global_ind_end:, 0] = Sol.Model.Nodal.V0
    Coup_A[global_ind_end:, 0] = Sol.Model.Nodal.A0

    # Uncoupled system matrices
    UnCoup_Kg = sparse.lil_matrix((Coup_DOF_Tnum, Coup_DOF_Tnum))
    UnCoup_Cg = sparse.lil_matrix((Coup_DOF_Tnum, Coup_DOF_Tnum))
    UnCoup_Mg = sparse.lil_matrix((Coup_DOF_Tnum, Coup_DOF_Tnum))
    UnCoup_F = np.zeros(Coup_DOF_Tnum)

    for veh_num in range(num_veh):
        veh = Veh_list[veh_num]
        gi = veh.global_ind
        ix = np.ix_(gi, gi)
        UnCoup_Kg[ix] = veh.SysM.K
        UnCoup_Cg[ix] = veh.SysM.C
        UnCoup_Mg[ix] = veh.SysM.M
        UnCoup_F[gi] = veh.SysM.M @ (veh.DOF.vert * grav)

    sl = slice(global_ind_end, None)
    UnCoup_Kg[sl, sl] = Model.Mesh.Kg
    UnCoup_Cg[sl, sl] = Model.Mesh.Cg
    UnCoup_Mg[sl, sl] = Model.Mesh.Mg

    UnCoup_Kg = sparse.csc_matrix(UnCoup_Kg)
    UnCoup_Cg = sparse.csc_matrix(UnCoup_Cg)
    UnCoup_Mg = sparse.csc_matrix(UnCoup_Mg)

    disp_every = getattr(Calc.Options, 'disp_every', 2)
    start_time = time_module.time()
    last_display = disp_every

    # *** With VBI ***
    if Calc.Options.VBI == 1:
        for t in range(num_t - 1):
            elapsed = time_module.time() - start_time
            if elapsed > last_display:
                pct = round(t / num_t * 100, 2)
                print(f'Time step {t} of {num_t} ({pct}%)')
                last_display += disp_every

            # Time-dependent coupling terms - collect sparse triplets
            rows_diag = []
            cols_diag = []
            vals_Kg_diag = []
            vals_Cg_diag = []
            vals_Mg_diag = []

            rows_off = []
            cols_off = []
            vals_Kg_off = []
            vals_Cg_off = []

            F_add = np.zeros(Coup_DOF_Tnum)

            for veh_num in range(num_veh):
                veh = Veh_list[veh_num]
                ks = veh.ktn    # per-wheel primary stiffness
                cs = veh.ctn    # per-wheel primary damping
                ms = veh.mtn    # per-wheel masses
                cv = Calc.Veh[veh_num]
                N2w_wheels = veh.Wheels.N2w
                veh_gi = veh.global_ind
                num_veh_dof = len(veh_gi)

                for wheel in range(veh.Wheels.num):
                    ele_num = cv.elexj[wheel, t + 1]
                    if ele_num < 0:
                        continue

                    x = cv.xj[wheel, t + 1]
                    a = Track.Rail.Mesh.Ele.a[ele_num]

                    sfx = shape_fun(x, a).flatten()
                    sfxp = shape_fun_p(x, a).flatten()
                    sfxpp = shape_fun_pp(x, a).flatten()

                    NN = np.outer(sfx, sfx)
                    NNp = np.outer(sfx, sfxp)
                    NNpp = np.outer(sfx, sfxpp)

                    col_dof = global_ind_end + ele_DOF[ele_num, :]

                    # Diagonal block addition
                    Kg_add = NN * ks[wheel] + cs[wheel] * vel * NNp + ms[wheel] * vel2 * NNpp
                    Cg_add = NN * cs[wheel] + 2 * ms[wheel] * vel * NNp
                    Mg_add = NN * ms[wheel]

                    for ii in range(4):
                        for jj in range(4):
                            rows_diag.append(col_dof[ii])
                            cols_diag.append(col_dof[jj])
                            vals_Kg_diag.append(Kg_add[ii, jj])
                            vals_Cg_diag.append(Cg_add[ii, jj])
                            vals_Mg_diag.append(Mg_add[ii, jj])

                    # Off-diagonal block matrices
                    N2w = N2w_wheels[wheel, :]
                    OffDiag = -np.outer(sfx, N2w)
                    OffDiag_d = -np.outer(sfxp, N2w) * vel

                    Kg_off_v2t = (OffDiag * ks[wheel] + OffDiag_d * cs[wheel]).T
                    Kg_off_t2v = (OffDiag * ks[wheel]).T
                    Cg_off = (OffDiag * cs[wheel]).T

                    # rows=veh_gi, cols=col_dof (vehicle->track)
                    for ii in range(num_veh_dof):
                        for jj in range(4):
                            rows_off.append(veh_gi[ii])
                            cols_off.append(col_dof[jj])
                            vals_Kg_off.append(Kg_off_v2t[ii, jj])
                            vals_Cg_off.append(Cg_off[ii, jj])

                    # rows=col_dof, cols=veh_gi (track->vehicle)
                    for jj in range(4):
                        for ii in range(num_veh_dof):
                            rows_off.append(col_dof[jj])
                            cols_off.append(veh_gi[ii])
                            vals_Kg_off.append(Kg_off_t2v[ii, jj])
                            vals_Cg_off.append(Cg_off[ii, jj])

                    # Force vector
                    h_path = cv.h_path[wheel, t + 1]
                    hd_path = cv.hd_path[wheel, t + 1]
                    hdd_path = cv.hdd_path[wheel, t + 1]

                    F_add[col_dof] += (ms[wheel] * grav - ms[wheel] * hdd_path) * sfx
                    f_profile = ks[wheel] * h_path + cs[wheel] * hd_path
                    F_add[veh_gi] += f_profile * N2w
                    F_add[col_dof] += -f_profile * sfx

            # Assemble coupling sparse matrices without dynamic format conversion overhead
            # Filter boundary fixed DOFs directly from triplets before sparse matrix assembly
            if len(rows_diag) > 0:
                # Filter out diagonal entries mapping to fixed DOFs
                valid_diag = [idx for idx, r in enumerate(rows_diag) if r not in BC_DOF_fixed and cols_diag[idx] not in BC_DOF_fixed]
                r_diag_f = [rows_diag[idx] for idx in valid_diag]
                c_diag_f = [cols_diag[idx] for idx in valid_diag]
                
                Kg_coup_diag = sparse.csc_matrix(
                    ([vals_Kg_diag[idx] for idx in valid_diag], (r_diag_f, c_diag_f)),
                    shape=(Coup_DOF_Tnum, Coup_DOF_Tnum))
                Cg_coup_diag = sparse.csc_matrix(
                    ([vals_Cg_diag[idx] for idx in valid_diag], (r_diag_f, c_diag_f)),
                    shape=(Coup_DOF_Tnum, Coup_DOF_Tnum))
                Mg_coup_diag = sparse.csc_matrix(
                    ([vals_Mg_diag[idx] for idx in valid_diag], (r_diag_f, c_diag_f)),
                    shape=(Coup_DOF_Tnum, Coup_DOF_Tnum))
            else:
                z = sparse.csc_matrix((Coup_DOF_Tnum, Coup_DOF_Tnum))
                Kg_coup_diag = z; Cg_coup_diag = z; Mg_coup_diag = z

            if len(rows_off) > 0:
                # Filter out off-diagonal entries mapping to fixed DOFs
                valid_off = [idx for idx, r in enumerate(rows_off) if r not in BC_DOF_fixed and cols_off[idx] not in BC_DOF_fixed]
                r_off_f = [rows_off[idx] for idx in valid_off]
                c_off_f = [cols_off[idx] for idx in valid_off]
                
                Kg_coup_off = sparse.csc_matrix(
                    ([vals_Kg_off[idx] for idx in valid_off], (r_off_f, c_off_f)),
                    shape=(Coup_DOF_Tnum, Coup_DOF_Tnum))
                Cg_coup_off = sparse.csc_matrix(
                    ([vals_Cg_off[idx] for idx in valid_off], (r_off_f, c_off_f)),
                    shape=(Coup_DOF_Tnum, Coup_DOF_Tnum))
            else:
                z = sparse.csc_matrix((Coup_DOF_Tnum, Coup_DOF_Tnum))
                Kg_coup_off = z; Cg_coup_off = z

            Coup_Kg = UnCoup_Kg + Kg_coup_diag + Kg_coup_off
            Coup_Cg = UnCoup_Cg + Cg_coup_diag + Cg_coup_off
            Coup_Mg = UnCoup_Mg + Mg_coup_diag
            
            Coup_F = UnCoup_F + F_add
            Coup_F[BC_DOF_fixed] = 0

            # Newmark-Beta integration
            effKg = Coup_Kg + NB[0] * Coup_Mg + NB[1] * Coup_Cg
            A_vec = Coup_U[:, t] * NB[0] + Coup_V[:, t] * NB[2] + Coup_A[:, t] * NB[3]
            B_vec = NB[1] * Coup_U[:, t] - NB[4] * Coup_V[:, t] - NB[5] * Coup_A[:, t]
            rhs = Coup_F + Coup_Mg @ A_vec + Coup_Cg @ B_vec
            Coup_U[:, t + 1] = spsolve(effKg, rhs)
            Coup_V[:, t + 1] = NB[1] * Coup_U[:, t + 1] - B_vec
            Coup_A[:, t + 1] = Coup_U[:, t + 1] * NB[0] - A_vec

    # *** Moving Force (no VBI) ***
    elif Calc.Options.VBI == 0:
        print('No VBI!!!')

        Coup_Kg = UnCoup_Kg.copy()
        Coup_Cg = UnCoup_Cg.copy()
        Coup_Mg = UnCoup_Mg.copy()

        effKg = Coup_Kg + NB[0] * Coup_Mg + NB[1] * Coup_Cg

        for t in range(num_t - 1):
            elapsed = time_module.time() - start_time
            if elapsed > last_display:
                pct = round(t / num_t * 100, 2)
                print(f'Time step {t} of {num_t} ({pct}%)')
                last_display += disp_every

            Coup_F = UnCoup_F.copy()

            for veh_num in range(num_veh):
                veh = Veh_list[veh_num]
                cv = Calc.Veh[veh_num]
                for wheel in range(veh.Wheels.num):
                    ele_num = cv.elexj[wheel, t + 1]
                    if ele_num < 0:
                        continue
                    x = cv.xj[wheel, t + 1]
                    a = Track.Rail.Mesh.Ele.a[ele_num]
                    sfx = shape_fun(x, a).flatten()
                    col_dof = global_ind_end + ele_DOF[ele_num, :]
                    Coup_F[col_dof] = veh.sta_loads[wheel] * sfx

            Coup_F[BC_DOF_fixed] = 0

            A_vec = Coup_U[:, t] * NB[0] + Coup_V[:, t] * NB[2] + Coup_A[:, t] * NB[3]
            B_vec = NB[1] * Coup_U[:, t] - NB[4] * Coup_V[:, t] - NB[5] * Coup_A[:, t]
            rhs = Coup_F + Coup_Mg @ A_vec + Coup_Cg @ B_vec
            Coup_U[:, t + 1] = spsolve(effKg, rhs)
            Coup_V[:, t + 1] = NB[1] * Coup_U[:, t + 1] - B_vec
            Coup_A[:, t + 1] = Coup_U[:, t + 1] * NB[0] - A_vec

    # ---- Output generation ----
    # Pre-merge moving train simulation stage
    for veh_num in range(num_veh):
        gi = Veh_list[veh_num].global_ind
        Sol.Veh[veh_num].U = Coup_U[gi, :]
        Sol.Veh[veh_num].V = Coup_V[gi, :]
        Sol.Veh[veh_num].A = Coup_A[gi, :]
    Sol.Model.Nodal.U = Coup_U[global_ind_end:, :]
    Sol.Model.Nodal.V = Coup_V[global_ind_end:, :]
    Sol.Model.Nodal.A = Coup_A[global_ind_end:, :]

    # ── Tail Simulation Stage (Free Vibration Decay) ──
    # Integrate system matrices with zero input force for an additional 10.0s using a coarse step (dt_tail = 0.01s)
    dt_tail = 0.01
    tail_duration = 10.0
    num_t_tail = int(tail_duration / dt_tail)

    NB_tail = np.zeros(6)
    NB_tail[0] = 1.0 / (beta * dt_tail ** 2)
    NB_tail[1] = delta / (beta * dt_tail)
    NB_tail[2] = 1.0 / (beta * dt_tail)
    NB_tail[3] = 1.0 / (2 * beta) - 1.0
    NB_tail[4] = 1.0 - delta / beta
    NB_tail[5] = (1.0 - delta / (2 * beta)) * dt_tail

    effKg_tail = UnCoup_Kg + NB_tail[0] * UnCoup_Mg + NB_tail[1] * UnCoup_Cg
    effKg_tail = sparse.csc_matrix(effKg_tail)

    # Initialize tail state from last frame of moving load stage
    U_tail = np.zeros((Coup_DOF_Tnum, num_t_tail))
    V_tail = np.zeros((Coup_DOF_Tnum, num_t_tail))
    A_tail = np.zeros((Coup_DOF_Tnum, num_t_tail))

    u_curr = Coup_U[:, -1].copy()
    v_curr = Coup_V[:, -1].copy()
    a_curr = Coup_A[:, -1].copy()

    for t in range(num_t_tail):
        A_vec = u_curr * NB_tail[0] + v_curr * NB_tail[2] + a_curr * NB_tail[3]
        B_vec = NB_tail[1] * u_curr - NB_tail[4] * v_curr - NB_tail[5] * a_curr
        rhs = UnCoup_Mg @ A_vec + UnCoup_Cg @ B_vec
        rhs[BC_DOF_fixed] = 0

        u_next = spsolve(effKg_tail, rhs)
        v_next = NB_tail[1] * u_next - B_vec
        a_next = u_next * NB_tail[0] - A_vec

        U_tail[:, t] = u_next
        V_tail[:, t] = v_next
        A_tail[:, t] = a_next

        u_curr, v_curr, a_curr = u_next, v_next, a_next

    # Concatenate final structural trajectories
    Sol.Model.Nodal.U = np.column_stack([Sol.Model.Nodal.U, U_tail[global_ind_end:, :]])
    Sol.Model.Nodal.V = np.column_stack([Sol.Model.Nodal.V, V_tail[global_ind_end:, :]])
    Sol.Model.Nodal.A = np.column_stack([Sol.Model.Nodal.A, A_tail[global_ind_end:, :]])

    # Pad vehicle solution trajectories with terminal zeros to keep array dimensions globally uniform
    for veh_num in range(num_veh):
        n_vdof = Sol.Veh[veh_num].U.shape[0]
        z_pad  = np.zeros((n_vdof, num_t_tail))
        Sol.Veh[veh_num].U = np.column_stack([Sol.Veh[veh_num].U, z_pad])
        Sol.Veh[veh_num].V = np.column_stack([Sol.Veh[veh_num].V, z_pad])
        Sol.Veh[veh_num].A = np.column_stack([Sol.Veh[veh_num].A, z_pad])

    # Store auxiliary tail timing sequence inside solver for plotting alignment
    t_last = Calc.Solver.t[-1]
    t_tail_arr = t_last + np.arange(1, num_t_tail + 1) * dt_tail
    Calc.Solver.t = np.concatenate([Calc.Solver.t, t_tail_arr])
    Calc.Solver.num_t = len(Calc.Solver.t)

    return Sol
