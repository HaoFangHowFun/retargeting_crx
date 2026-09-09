import numpy as np
import pytest

from retargeting_ros.dual_crx_contract import (
    BIMANUAL_CRX_NAMES,
    CRX_PROFILE_NAMES,
    DUAL_CRX_NAMES,
    to_dual_crx_command,
    to_dual_crx_names,
    to_dual_crx_positions,
    to_bimanual_crx_command,
    to_bimanual_crx_positions,
    split_bimanual_crx_positions,
    join_bimanual_crx_positions,
)


def test_maps_profile_names_to_dual_crx_names():
    assert to_dual_crx_names(CRX_PROFILE_NAMES) == DUAL_CRX_NAMES
    assert DUAL_CRX_NAMES[:6] == tuple(f"right_J{i}" for i in range(1, 7))
    assert DUAL_CRX_NAMES[6:] == tuple(f"right_leap_joint_{i}" for i in range(16))


def test_rejects_unexpected_profile_order():
    with pytest.raises(ValueError, match="exactly"):
        to_dual_crx_names(tuple(reversed(CRX_PROFILE_NAMES)))


@pytest.mark.parametrize("qpos", [np.zeros(21), np.zeros(23), np.full(22, np.nan), np.full(22, np.inf)])
def test_rejects_invalid_positions(qpos):
    with pytest.raises(ValueError):
        to_dual_crx_positions(qpos)


def test_command_is_finite_and_detached():
    qpos = np.arange(22, dtype=float)
    names, values = to_dual_crx_command(qpos, names=CRX_PROFILE_NAMES)
    assert names == DUAL_CRX_NAMES
    assert values == tuple(qpos.tolist())


def test_bimanual_command_uses_left_then_right_order():
    qpos = np.arange(44, dtype=float)
    names, values = to_bimanual_crx_command(qpos)
    assert names == BIMANUAL_CRX_NAMES
    assert names[:6] == tuple(f"left_J{i}" for i in range(1, 7))
    assert names[6:22] == tuple(f"left_leap_joint_{i}" for i in range(16))
    assert names[22:28] == tuple(f"right_J{i}" for i in range(1, 7))
    assert names[28:] == tuple(f"right_leap_joint_{i}" for i in range(16))
    assert values == tuple(qpos.tolist())
    left, right = split_bimanual_crx_positions(qpos)
    assert left == tuple(qpos[:22].tolist())
    assert right == tuple(qpos[22:].tolist())
    assert join_bimanual_crx_positions(left, right) == values


@pytest.mark.parametrize("qpos", [np.zeros(43), np.zeros(45), np.full(44, np.nan)])
def test_rejects_invalid_bimanual_positions(qpos):
    with pytest.raises(ValueError):
        to_bimanual_crx_positions(qpos)


def test_publisher_module_is_importable_without_ros():
    from retargeting_ros.dual_crx_publisher import BimanualDualCrxPublisher, DualCrxPublisher

    assert DualCrxPublisher.__name__ == "DualCrxPublisher"
    assert BimanualDualCrxPublisher.__name__ == "BimanualDualCrxPublisher"
