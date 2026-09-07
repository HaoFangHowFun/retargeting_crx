"""Passive mjviser adapter for an application-owned MuJoCo loop."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

import numpy as np

from retargeting_apps.config import MujocoWebViewerConfig
from retargeting_apps.visualization.viser_scene import (
    ViserHandObservationRenderer,
    rotation_matrix_to_wxyz,
)
from retargeting.core.types import RetargetingHandObservation


_JOINT_ANGLE_DECIMALS = 5
_WRIST_ERROR_DECIMALS = 3
_MJVISER_WORLD_FRAME = "/fixed_bodies"
_ROBOT_WRIST_NODE = f"{_MJVISER_WORLD_FRAME}/current/robot/wrist"
_TARGET_WRIST_NODE = f"{_MJVISER_WORLD_FRAME}/current/human/wrist"
_ROBOT_WRIST_AXES_LENGTH = 0.08
_ROBOT_WRIST_AXES_RADIUS = 0.005
_WRIST_LABEL_OFFSET = (0.0, 0.0, 0.075)


def _wrist_position_error_cm(target_pose: np.ndarray, actual_pose: np.ndarray) -> float:
    """Return Euclidean wrist-origin error in centimetres."""
    delta = np.asarray(actual_pose, dtype=float)[:3, 3] - np.asarray(target_pose, dtype=float)[:3, 3]
    return float(np.linalg.norm(delta) * 100.0)


def _wrist_orientation_error_deg(target_pose: np.ndarray, actual_pose: np.ndarray) -> float:
    """Return the shortest target-to-actual wrist rotation angle in degrees."""
    target_rotation = np.asarray(target_pose, dtype=float)[:3, :3]
    actual_rotation = np.asarray(actual_pose, dtype=float)[:3, :3]
    relative_rotation = target_rotation.T @ actual_rotation
    cosine = np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def _load_mjviser_dependencies() -> tuple[Any, Any, Any]:
    """Load optional Web viewer dependencies only for an enabled viewer.

    Args:
        None.

    Returns:
        The MuJoCo module, viser module, and mjviser scene class.
    """
    try:
        import mujoco
        import viser
        from mjviser import ViserMujocoScene
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "MuJoCo Web visualization is not installed. "
            "Install it with `pip install -e \".[mujoco-web]\"`."
        ) from exc
    return mujoco, viser, ViserMujocoScene


class MujocoWebVisualizer:
    """Publish one externally stepped MuJoCo simulation through mjviser."""

    def __init__(
        self,
        model: Any,
        data: Any,
        config: MujocoWebViewerConfig,
        wrist_frame_name: str = "wrist",
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        """Create a passive scene around an existing MuJoCo model and data.

        Args:
            model: MuJoCo model owned by the execution backend.
            data: Live MuJoCo data owned and stepped by the execution backend.
            config: Validated Web viewer configuration.
            wrist_frame_name: MuJoCo body used as the actual robot wrist frame.
            sleep: Wait dependency used for client and completion waits.

        Returns:
            None.
        """
        config.validate()
        mujoco, viser, scene_class = _load_mjviser_dependencies()
        self.config = config
        self.model = model
        self.data = data
        self.wrist_frame_name = str(wrist_frame_name)
        self._sleep = sleep
        self._closed = False
        self._target_wrist_pose_world: np.ndarray | None = None
        self._robot_wrist_handle: Any | None = None
        self._robot_wrist_label_handle: Any | None = None
        self._target_wrist_label_handle: Any | None = None
        self.server = viser.ViserServer(host=config.host, port=config.port)
        try:
            self.scene = scene_class(self.server, model, num_envs=1)
            tabs = self.scene.create_visualization_gui(
                camera_distance=config.camera_distance,
                camera_azimuth=config.camera_azimuth,
                camera_elevation=config.camera_elevation,
            )
            self._wrist_body_id = self._resolve_wrist_body_id(mujoco)
            self._joint_angle_handles = self._create_joint_angle_gui(mujoco, viser, tabs)
            (
                self._wrist_position_error_handle,
                self._wrist_orientation_error_handle,
            ) = self._create_wrist_diagnostics_gui(viser, tabs)
            self._hand_renderer = ViserHandObservationRenderer(
                self.server,
                point_size=config.human_keypoint_size,
                # mjviser applies its camera-tracking scene offset to this frame.
                root_node_name=f"{_MJVISER_WORLD_FRAME}/current/human",
            )
        except Exception:
            self.server.stop()
            raise
        print(f"mjviser server listening on {config.host}:{self.server.get_port()}.")

    def _resolve_wrist_body_id(self, mujoco: Any) -> int:
        """Resolve the configured robot wrist as a MuJoCo body."""
        body_id = int(
            mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                self.wrist_frame_name,
            )
        )
        if body_id < 0:
            raise ValueError(
                f"MuJoCo model is missing wrist body {self.wrist_frame_name!r}."
            )
        return body_id

    def _create_joint_angle_gui(self, mujoco: Any, viser: Any, tabs: Any) -> list[tuple[Any, int]]:
        """Create read-only angle fields for every hinge joint in model order.

        Args:
            mujoco: Imported MuJoCo module used for joint enums and names.
            viser: Imported Viser module used for the tab icon.
            tabs: Existing mjviser tab group extended by this adapter.

        Returns:
            Pairs of GUI number handles and their corresponding qpos addresses.
        """
        handles: list[tuple[Any, int]] = []
        hinge_type = int(mujoco.mjtJoint.mjJNT_HINGE)
        with tabs.add_tab("Joint angles", icon=viser.Icon.ADJUSTMENTS):
            for joint_id in range(self.model.njnt):
                if int(self.model.jnt_type[joint_id]) != hinge_type:
                    continue
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                if not name:
                    name = f"joint_{joint_id}"
                qpos_address = int(self.model.jnt_qposadr[joint_id])
                value = round(float(self.data.qpos[qpos_address]), _JOINT_ANGLE_DECIMALS)
                handle = self.server.gui.add_number(
                    f"{name} [rad]",
                    initial_value=value,
                    step=10.0 ** -_JOINT_ANGLE_DECIMALS,
                    disabled=True,
                    hint="Current MuJoCo joint angle in radians.",
                )
                handles.append((handle, qpos_address))
        return handles

    def _create_wrist_diagnostics_gui(self, viser: Any, tabs: Any) -> tuple[Any, Any]:
        """Create read-only target-to-actual wrist error fields."""
        with tabs.add_tab("Wrist diagnostics", icon=viser.Icon.ADJUSTMENTS):
            position_handle = self.server.gui.add_number(
                "Position error [cm]",
                initial_value=0.0,
                step=10.0 ** -_WRIST_ERROR_DECIMALS,
                disabled=True,
                hint="Distance between the Quest target and actual robot wrist origins.",
            )
            orientation_handle = self.server.gui.add_number(
                "Orientation error [deg]",
                initial_value=0.0,
                step=10.0 ** -_WRIST_ERROR_DECIMALS,
                disabled=True,
                hint="Shortest rotation angle between the Quest target and actual robot wrist frames.",
            )
        return position_handle, orientation_handle

    def _actual_wrist_pose(self) -> np.ndarray:
        """Read the actual robot wrist body pose from current MuJoCo data."""
        pose = np.eye(4, dtype=float)
        pose[:3, :3] = np.asarray(
            self.data.xmat[self._wrist_body_id], dtype=float
        ).reshape(3, 3)
        pose[:3, 3] = np.asarray(
            self.data.xpos[self._wrist_body_id], dtype=float
        )
        return pose

    def _update_wrist_diagnostics(self) -> None:
        """Publish the actual wrist frame and current target error values."""
        actual_pose = self._actual_wrist_pose()
        with self.server.atomic():
            if self._robot_wrist_handle is None:
                self._robot_wrist_handle = self.server.scene.add_frame(
                    _ROBOT_WRIST_NODE,
                    wxyz=rotation_matrix_to_wxyz(actual_pose[:3, :3]),
                    position=actual_pose[:3, 3],
                    axes_length=_ROBOT_WRIST_AXES_LENGTH,
                    axes_radius=_ROBOT_WRIST_AXES_RADIUS,
                    origin_color=(255, 0, 255),
                )
                if hasattr(self.server.scene, "add_label"):
                    self._robot_wrist_label_handle = self.server.scene.add_label(
                        f"{_ROBOT_WRIST_NODE}/label",
                        "Panda wrist (actual)",
                        position=_WRIST_LABEL_OFFSET,
                    )
            else:
                self._robot_wrist_handle.wxyz = rotation_matrix_to_wxyz(
                    actual_pose[:3, :3]
                )
                self._robot_wrist_handle.position = actual_pose[:3, 3]
                self._robot_wrist_handle.visible = True
            if self._target_wrist_pose_world is not None:
                self._wrist_position_error_handle.value = round(
                    _wrist_position_error_cm(
                        self._target_wrist_pose_world,
                        actual_pose,
                    ),
                    _WRIST_ERROR_DECIMALS,
                )
                self._wrist_orientation_error_handle.value = round(
                    _wrist_orientation_error_deg(
                        self._target_wrist_pose_world,
                        actual_pose,
                    ),
                    _WRIST_ERROR_DECIMALS,
                )

    def _update_joint_angles(self) -> None:
        """Atomically refresh all joint-angle fields from the live qpos state.

        Args:
            None.

        Returns:
            None.
        """
        with self.server.atomic():
            for handle, qpos_address in self._joint_angle_handles:
                value = round(float(self.data.qpos[qpos_address]), _JOINT_ANGLE_DECIMALS)
                if handle.value != value:
                    handle.value = value

    def update(self) -> None:
        """Publish the backend's current MuJoCo state without stepping it.

        Args:
            None.

        Returns:
            None.
        """
        self.scene.update_from_mjdata(self.data)
        self._update_joint_angles()
        self._update_wrist_diagnostics()

    def update_observation(self, observation: RetargetingHandObservation) -> None:
        """Publish one canonical human-hand observation in the MuJoCo scene.

        Args:
            observation: Canonical hand observation produced by the mapping layer.

        Returns:
            None.
        """
        self._target_wrist_pose_world = np.asarray(
            observation.wrist_pose_world,
            dtype=float,
        ).copy()
        self._hand_renderer.update_observation(observation)
        if self._target_wrist_label_handle is None and hasattr(
            self.server.scene, "add_label"
        ):
            self._target_wrist_label_handle = self.server.scene.add_label(
                f"{_TARGET_WRIST_NODE}/label",
                "Quest wrist (target)",
                position=_WRIST_LABEL_OFFSET,
            )
        elif self._target_wrist_label_handle is not None:
            self._target_wrist_label_handle.visible = True
        self._update_wrist_diagnostics()

    def hide_observation(self) -> None:
        """Hide the current human hand when no valid observation is available.

        Args:
            None.

        Returns:
            None.
        """
        self._target_wrist_pose_world = None
        self._hand_renderer.hide()
        if self._target_wrist_label_handle is not None:
            self._target_wrist_label_handle.visible = False

    def wait_for_client(self) -> None:
        """Optionally wait until at least one browser has connected.

        Args:
            None.

        Returns:
            None.
        """
        if not self.config.wait_for_client:
            return
        print("Waiting for an mjviser browser client before starting simulation.")
        while not self.server.get_clients():
            self._sleep(0.05)

    def wait_after_completion(self) -> None:
        """Optionally retain the final scene until the user interrupts it.

        Args:
            None.

        Returns:
            None.
        """
        if not self.config.keep_open_after_completion:
            return
        print("Offline simulation complete; press Ctrl+C to stop the mjviser server.")
        try:
            while True:
                self._sleep(1.0)
        except KeyboardInterrupt:
            pass

    def close(self) -> None:
        """Stop the Viser server exactly once.

        Args:
            None.

        Returns:
            None.
        """
        if self._closed:
            return
        self._closed = True
        self.server.stop()


def create_mujoco_web_visualizer(
    model: Any,
    data: Any,
    config: MujocoWebViewerConfig,
    *,
    wrist_frame_name: str = "wrist",
) -> MujocoWebVisualizer:
    """Create the concrete passive mjviser adapter.

    Args:
        model: MuJoCo model owned by the execution backend.
        data: Live MuJoCo data owned by the execution backend.
        config: Validated Web viewer configuration.
        wrist_frame_name: MuJoCo body used as the actual robot wrist frame.

    Returns:
        Ready passive MuJoCo Web visualizer.
    """
    return MujocoWebVisualizer(
        model=model,
        data=data,
        config=config,
        wrist_frame_name=wrist_frame_name,
    )


__all__ = ["MujocoWebVisualizer", "create_mujoco_web_visualizer"]
