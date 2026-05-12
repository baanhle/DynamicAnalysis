"""
B00 - Calculations
Core orchestrator for TTB-2D simulation. Equivalent to B00_Calculations.m.
"""
import time as time_module
import numpy as np

from .B43_model_geometry import B43_ModelGeometry
from .B07_options_processing import B07_OptionsProcessing
from .B01_elements_and_coordinates import B01_ElementsAndCoordinates
from .B02_boundary_conditions import B02_BoundaryConditions
from .B03_beam_matrices import B03_BeamMatrices
from .B09_beam_frq import B09_BeamFrq
from .B24_beam_damping import B24_BeamDamping
from .B51_rail_variables import B51_RailVariables
from .B54_model_matrices import B54_ModelMatrices
from .B55_model_bc import B55_ModelBC
from .B56_model_frq import B56_ModelFrq
from .B18_train_veh_eq import B18_TrainVehEq
from .B08_veh_freq import B08_VehFreq
from .B10_end_time import B10_EndTime
from .B11_time_space_discretization import B11_TimeSpaceDiscretization
from .B47_veh_static_loads import B47_VehStaticLoads
from .B19_generate_profile import B19_GenerateProfile
from .B25_wheel_profiles import B25_WheelProfiles
from .B50_element_num_of_force import B50_ElementNumOfForce
from .B64_coupled_initial_static import B64_Coupled_InitialStatic
from .B65_dynamic_calc_coupled_faster import B65_DynamicCalcCoupledFaster
from .B66_contact_force import B66_ContactForce
from .B49_beam_deformation import B49_BeamDeformation
from .B31_beam_bm import B31_BeamBM
from .B33_beam_shear import B33_BeamShear
from .B53_beam_acceleration import B53_BeamAcceleration
from .B58_results_beam_sections import B58_ResultsBeamSections


def B00_Calculations(Calc, Train, Track, Beam):
    """
    Run the full TTB-2D simulation.

    Parameters
    ----------
    Calc, Train, Track, Beam : SimpleNamespace objects with required fields.

    Returns
    -------
    Calc, Train, Track, Beam, Sol : Updated structures with solution.
    """
    # -- Model geometry --
    Calc, Train, Beam = B43_ModelGeometry(Calc, Train, Track, Beam)

    # -- Options processing --
    Calc, Train, Track, Beam = B07_OptionsProcessing(Calc, Train, Track, Beam)

    # ---- Beam Model ----
    Beam = B01_ElementsAndCoordinates(Beam, Calc)
    Beam = B02_BoundaryConditions(Beam)
    Beam = B03_BeamMatrices(Beam)
    Beam = B09_BeamFrq(Beam, Calc)
    Beam = B24_BeamDamping(Beam)

    # ---- Track Model ----
    Track = B51_RailVariables(Track, Calc)
    Track.Rail = B01_ElementsAndCoordinates(Track.Rail)
    Track.Rail = B02_BoundaryConditions(Track.Rail)
    Track.Rail = B03_BeamMatrices(Track.Rail)
    Track.Rail = B09_BeamFrq(Track.Rail, Calc)
    # Reference frequencies for damping
    Track.Rail.Modal.w = np.concatenate([
        np.zeros(Track.Rail.Modal.num_rigid_modes),
        Beam.Modal.w[:2]
    ])
    Track.Rail = B24_BeamDamping(Track.Rail)

    # ---- Complete Model ----
    print('Building model system matrices ...')
    Model = B54_ModelMatrices(Beam, Track, Calc)
    Model = B55_ModelBC(Model, Beam, Track)
    Model = B56_ModelFrq(Model, Calc)
    print(' DONE')

    # ---- Vehicle Model ----
    Veh_list = Train.Veh.data
    for i, veh in enumerate(Veh_list):
        Veh_list[i] = B18_TrainVehEq(veh)

    # Vehicle frequencies (as list)
    B08_VehFreq(Veh_list, Calc)

    # Vehicle positions and solver time array
    Calc = B10_EndTime(Calc)
    Calc, Beam, Train, Track = B11_TimeSpaceDiscretization(Calc, Beam, Train, Track)[:4]

    # Vehicle static loads
    B47_VehStaticLoads(Veh_list, Calc)

    # ---- Irregularity profile ----
    Calc, Beam, Train = B19_GenerateProfile(Calc, Beam, Train)
    Calc = B25_WheelProfiles(Calc, Veh_list)

    # ---- Set global indices for vehicles ----
    dof_offset = 0
    for veh in Veh_list:
        veh.global_ind = np.arange(dof_offset, dof_offset + veh.Tnum_DOF, dtype=int)
        veh.vel = Train.vel
        dof_offset += veh.Tnum_DOF

    # ---- Coupled Equations Solver ----
    # Element number of vertical forces in time
    Calc = B50_ElementNumOfForce(Track.Rail, Calc)

    # Initial static deformation
    Sol = B64_Coupled_InitialStatic(Veh_list, Model, Calc, Track)

    t0 = time_module.time()
    print('Performing Dynamic Calculations (Coupled System) ...')
    Sol = B65_DynamicCalcCoupledFaster(Veh_list, Model, Calc, Track, Sol)
    print(f'Calculation time: {round(time_module.time() - t0, 2)}s')

    # ---- Additional results ----
    Sol = B66_ContactForce(Sol, Track, Calc, Train)
    Sol = B49_BeamDeformation(Sol, Model, Beam, Calc, Train, 1)
    Sol = B49_BeamDeformation(Sol, Model, Beam, Calc, Train, 0)
    Sol = B31_BeamBM(Sol, Model, Beam, Calc, 1)
    Sol = B31_BeamBM(Sol, Model, Beam, Calc, 0)
    Sol = B33_BeamShear(Sol, Model, Beam, Calc, 1)
    Sol = B33_BeamShear(Sol, Model, Beam, Calc, 0)
    Sol = B53_BeamAcceleration(Sol, Model, Beam, Calc)
    Sol = B58_ResultsBeamSections(Sol, Beam, Calc)

    print('All calculations finished successfully')

    return Calc, Train, Track, Beam, Model, Sol
