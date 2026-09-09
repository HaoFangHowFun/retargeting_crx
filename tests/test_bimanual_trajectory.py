import numpy as np
import pytest
import yaml

from retargeting_apps.bimanual_trajectory import generate_arm_trajectory
from retargeting_apps.config import resolve_project_path


@pytest.mark.parametrize("side", ["left", "right"])
def test_trajectory_tracks_closed_eef_path_and_holds_fingers(side):
    config = yaml.safe_load(resolve_project_path("configs/bimanual/crx5ia_coact_leap.yaml").read_text())
    result = generate_arm_trajectory(config[side], samples=17)
    assert result["position_error_m"].max() < 0.001
    assert result["rotation_error_rad"].max() < np.deg2rad(0.5)
    np.testing.assert_allclose(result["target"][0], result["target"][-1], atol=1e-10)
    assert np.linalg.norm(result["target"][8, :3, 3] - result["target"][0, :3, 3]) > 0.005
    home = np.asarray(result["robot"].initial_qpos)
    np.testing.assert_allclose(result["qpos"][:, 6:], np.tile(home[6:], (17, 1)))
    np.testing.assert_allclose(result["qpos"][0], home, atol=1e-8)


@pytest.mark.parametrize("samples,amplitude", [(2, .08), (17, 0), (17, float("nan")), (17, .3)])
def test_trajectory_rejects_invalid_parameters(samples, amplitude):
    with pytest.raises(ValueError):
        generate_arm_trajectory({}, samples, amplitude)
