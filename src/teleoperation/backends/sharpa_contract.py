"""ROS-free validation of the local Sharpa URDF joint contract."""

import numpy as np


SHARPA_SUFFIXES = (
    'thumb_CMC_FE', 'thumb_CMC_AA', 'thumb_MCP_FE', 'thumb_MCP_AA', 'thumb_IP',
    'index_MCP_FE', 'index_MCP_AA', 'index_PIP', 'index_DIP',
    'middle_MCP_FE', 'middle_MCP_AA', 'middle_PIP', 'middle_DIP',
    'ring_MCP_FE', 'ring_MCP_AA', 'ring_PIP', 'ring_DIP',
    'pinky_CMC', 'pinky_MCP_FE', 'pinky_MCP_AA', 'pinky_PIP', 'pinky_DIP',
)


def sharpa_names(side):
    if side not in ('left', 'right'):
        raise ValueError('side must be left or right')
    return tuple(f'{side}_{suffix}' for suffix in SHARPA_SUFFIXES)


def joint_channels(robot_names):
    """Return named ROS groups and indices into left-then-right profile qpos."""
    if len(robot_names) != 2 or len(robot_names[0]) != len(robot_names[1]):
        raise ValueError('Expected matching left and right Sharpa robot dimensions')
    size = len(robot_names[0])
    if size not in (22, 28):
        raise ValueError('Sharpa profiles require 22 hand joints or 6 arm + 22 hand joints')
    arm_dof = size - 22
    channels, arm_names, arm_indices = {}, [], []
    for side, names, offset in zip(('left', 'right'), robot_names, (0, size)):
        names = tuple(names)
        expected_arm = tuple(f'J{i}' for i in range(1, 7)) if arm_dof else ()
        if (names[:arm_dof] != expected_arm or len(set(names)) != size
                or set(names[arm_dof:]) != set(sharpa_names(side))):
            raise ValueError(f'Invalid {side} Sharpa profile joint names')
        channels[f'{side}_hand'] = (
            sharpa_names(side), np.array([offset + names.index(n) for n in sharpa_names(side)]))
        arm_names.extend(f'{side}_{n}' for n in expected_arm)
        arm_indices.extend(range(offset, offset + arm_dof))
    if arm_dof:
        channels = {'arms': (tuple(arm_names), np.array(arm_indices)), **channels}
    return channels


def named_positions(names, positions, expected):
    values = np.asarray(positions, dtype=float)
    if (len(names) != len(expected) or len(set(names)) != len(expected)
            or set(names) != set(expected) or values.shape != (len(expected),)
            or not np.isfinite(values).all()):
        raise ValueError('Expected complete, distinct joint names and finite positions')
    by_name = dict(zip(names, values))
    return np.asarray([by_name[n] for n in expected])
