"""
B58 - Results at Beam Sections
Interpolates results at user-specified beam sections.
"""
import numpy as np


def B58_ResultsBeamSections(Sol, Beam, Calc):
    if Calc.Options.num_calc_beam_sections > 0:
        sections = np.atleast_1d(Calc.Options.calc_beam_sections)
        # Get all result field names under Sol.Beam
        for field_name in dir(Sol.Beam):
            field = getattr(Sol.Beam, field_name)
            if hasattr(field, 'xt'):
                result = []
                for sec in sections:
                    row = np.interp(sec, Beam.Mesh.Nodes.acum,
                                    field.xt[:, :].mean(axis=1) if field.xt.ndim == 2 else field.xt)
                    # Actually interpolate for each time step
                    pass
                # Proper interpolation per time step
                sections_t = np.zeros((len(sections), Calc.Solver.num_t))
                for i, sec in enumerate(sections):
                    for t_step in range(Calc.Solver.num_t):
                        sections_t[i, t_step] = np.interp(
                            sec, Beam.Mesh.Nodes.acum, field.xt[:, t_step])
                field.sections_t = sections_t

    return Sol
