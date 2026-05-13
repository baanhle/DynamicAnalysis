"""
B65 - Dynamic Calculation (Coupled System) - Faster version
Main Newmark-Beta time integration solver for the coupled vehicle-track-bridge system.
Uses sparse matrix assembly for time-dependent coupling terms.
"""
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import spsolve, splu
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
    bc_mask = np.zeros(Coup_DOF_Tnum, dtype=bool)
    bc_mask[BC_DOF_fixed] = True

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

    z_csc = sparse.csc_matrix((Coup_DOF_Tnum, Coup_DOF_Tnum))

    # *** With VBI ***
    if Calc.Options.VBI == 1:
        for t in range(num_t - 1):
            elapsed = time_module.time() - start_time
            if elapsed > last_display:
                pct = round(t / num_t * 100, 2)
                print(f'Time step {t} of {num_t} ({pct}%)')
                last_display += disp_every

            # Time-dependent coupling terms - collect sparse triplets
            rows_diag_blocks = []
            cols_diag_blocks = []
            vals_Kg_diag_blocks = []
            vals_Cg_diag_blocks = []
            vals_Mg_diag_blocks = []

            rows_off_blocks = []
            cols_off_blocks = []
            vals_Kg_off_blocks = []
            vals_Cg_off_blocks = []

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

                    rows_diag_blocks.append(np.repeat(col_dof, 4))
                    cols_diag_blocks.append(np.tile(col_dof, 4))
                    vals_Kg_diag_blocks.append(Kg_add.ravel())
                    vals_Cg_diag_blocks.append(Cg_add.ravel())
                    vals_Mg_diag_blocks.append(Mg_add.ravel())

                    # Off-diagonal block matrices
                    N2w = N2w_wheels[wheel, :]
                    OffDiag = -np.outer(sfx, N2w)
                    OffDiag_d = -np.outer(sfxp, N2w) * vel

                    Kg_off_v2t = (OffDiag * ks[wheel] + OffDiag_d * cs[wheel]).T
                    Kg_off_t2v = (OffDiag * ks[wheel]).T
                    Cg_off = (OffDiag * cs[wheel]).T

                    # rows=veh_gi, cols=col_dof (vehicle->track)
                    rows_off_blocks.append(np.repeat(veh_gi, 4))
                    cols_off_blocks.append(np.tile(col_dof, num_veh_dof))
                    vals_Kg_off_blocks.append(Kg_off_v2t.ravel())
                    vals_Cg_off_blocks.append(Cg_off.ravel())

                    # rows=col_dof, cols=veh_gi (track->vehicle)
                    rows_off_blocks.append(np.repeat(col_dof, num_veh_dof))
                    cols_off_blocks.append(np.tile(veh_gi, 4))
                    vals_Kg_off_blocks.append(Kg_off_t2v.ravel(order='F'))
                    vals_Cg_off_blocks.append(Cg_off.ravel(order='F'))

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
            if rows_diag_blocks:
                rows_diag_arr = np.concatenate(rows_diag_blocks).astype(int, copy=False)
                cols_diag_arr = np.concatenate(cols_diag_blocks).astype(int, copy=False)
                valid_diag = ~(bc_mask[rows_diag_arr] | bc_mask[cols_diag_arr])
                r_diag_f = rows_diag_arr[valid_diag]
                c_diag_f = cols_diag_arr[valid_diag]
                vkg_diag = np.concatenate(vals_Kg_diag_blocks).astype(float, copy=False)[valid_diag]
                vcg_diag = np.concatenate(vals_Cg_diag_blocks).astype(float, copy=False)[valid_diag]
                vmg_diag = np.concatenate(vals_Mg_diag_blocks).astype(float, copy=False)[valid_diag]

                Kg_coup_diag = sparse.csc_matrix(
                    (vkg_diag, (r_diag_f, c_diag_f)),
                    shape=(Coup_DOF_Tnum, Coup_DOF_Tnum))
                Cg_coup_diag = sparse.csc_matrix(
                    (vcg_diag, (r_diag_f, c_diag_f)),
                    shape=(Coup_DOF_Tnum, Coup_DOF_Tnum))
                Mg_coup_diag = sparse.csc_matrix(
                    (vmg_diag, (r_diag_f, c_diag_f)),
                    shape=(Coup_DOF_Tnum, Coup_DOF_Tnum))
            else:
                Kg_coup_diag = z_csc; Cg_coup_diag = z_csc; Mg_coup_diag = z_csc

            if rows_off_blocks:
                rows_off_arr = np.concatenate(rows_off_blocks).astype(int, copy=False)
                cols_off_arr = np.concatenate(cols_off_blocks).astype(int, copy=False)
                valid_off = ~(bc_mask[rows_off_arr] | bc_mask[cols_off_arr])
                r_off_f = rows_off_arr[valid_off]
                c_off_f = cols_off_arr[valid_off]
                vkg_off = np.concatenate(vals_Kg_off_blocks).astype(float, copy=False)[valid_off]
                vcg_off = np.concatenate(vals_Cg_off_blocks).astype(float, copy=False)[valid_off]

                Kg_coup_off = sparse.csc_matrix(
                    (vkg_off, (r_off_f, c_off_f)),
                    shape=(Coup_DOF_Tnum, Coup_DOF_Tnum))
                Cg_coup_off = sparse.csc_matrix(
                    (vcg_off, (r_off_f, c_off_f)),
                    shape=(Coup_DOF_Tnum, Coup_DOF_Tnum))
            else:
                Kg_coup_off = z_csc; Cg_coup_off = z_csc

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
                ks = veh.ktn
                cs = veh.ctn
                cv = Calc.Veh[veh_num]
                for wheel in range(veh.Wheels.num):
                    ele_num = cv.elexj[wheel, t + 1]
                    if ele_num < 0:
                        continue
                    x = cv.xj[wheel, t + 1]
                    a = Track.Rail.Mesh.Ele.a[ele_num]
                    sfx = shape_fun(x, a).flatten()
                    col_dof = global_ind_end + ele_DOF[ele_num, :]
                    # Moving force with profile perturbation: F = F_static - (ks*h + cs*hd)
                    h_path = cv.h_path[wheel, t + 1]
                    hd_path = cv.hd_path[wheel, t + 1]
                    f_wheel = veh.sta_loads[wheel] - (ks[wheel] * h_path + cs[wheel] * hd_path)
                    Coup_F[col_dof] += f_wheel * sfx

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
    dt_tail = float(getattr(Calc.Options, 'tail_dt', 0.01))
    tail_duration = float(getattr(Calc.Options, 'tail_duration', 10.0))
    num_t_tail = int(tail_duration / dt_tail)

    NB_tail = np.zeros(6)
    NB_tail[0] = 1.0 / (beta * dt_tail ** 2)
    NB_tail[1] = delta / (beta * dt_tail)
    NB_tail[2] = 1.0 / (beta * dt_tail)
    NB_tail[3] = 1.0 / (2 * beta) - 1.0
    NB_tail[4] = 1.0 - delta / beta
    NB_tail[5] = (1.0 - delta / (2 * beta)) * dt_tail

    effKg_tail = sparse.csc_matrix(UnCoup_Kg + NB_tail[0] * UnCoup_Mg + NB_tail[1] * UnCoup_Cg)
    effKg_tail_lu = splu(effKg_tail)

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

        u_next = effKg_tail_lu.solve(rhs)
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
