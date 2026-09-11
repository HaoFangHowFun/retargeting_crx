"""Measured hand angles may lie outside tighter retargeting profile bounds."""
from types import SimpleNamespace
import numpy as np
import pytest
from retargeting.core.optimizers.base import RetargetOptimizer


class QuadraticOptimizer(RetargetOptimizer):
    def get_objective_function(self, ref_values):
        def objective(x, gradient):
            difference = x - .2
            if gradient.size:
                gradient[:] = 2. * difference
            return float(difference @ difference)
        return objective


def optimizer():
    pytest.importorskip('nlopt')
    adaptor = SimpleNamespace(doa=4, robot_model=SimpleNamespace(joint_limits=np.array([[0., 2.]] * 4)),
                              backward_qpos=lambda q: q.copy())
    return QuadraticOptimizer(adaptor, solver='nlopt', solver_params={'ftol_abs': 1e-9})


def test_actual_negative_hand_angles_solve_without_mutating_feedback():
    opt = optimizer()
    measured = np.array([-.1349903663, -.0107378085, -.0199417194, -.0966407378])
    original = measured.copy()
    references = {'qpos_doa_last': measured}
    result = opt.retarget(references)
    np.testing.assert_allclose(result, .2, atol=1e-4)
    np.testing.assert_array_equal(measured, original)
    assert references['qpos_doa_last'] is measured
    np.testing.assert_array_equal(opt.joint_limits, [[0., 2.]] * 4)


@pytest.mark.parametrize('seed', [[0., 0.], [0., float('nan'), 0., 0.], [0., float('inf'), 0., 0.]])
def test_malformed_seed_remains_an_explicit_error(seed):
    with pytest.raises(ValueError, match='finite positions'):
        optimizer().retarget({'qpos_doa_last': seed})


def test_feasible_seed_is_unchanged_at_solver_boundary():
    opt = optimizer()
    seen = []
    opt.opt = SimpleNamespace(set_min_objective=lambda fn: None,
                              optimize=lambda x: (seen.append(x.copy()), x.copy())[1])
    seed = np.array([.1, .2, .3, .4])
    opt.retarget({'qpos_doa_last': seed})
    np.testing.assert_array_equal(seen[0], seed)
