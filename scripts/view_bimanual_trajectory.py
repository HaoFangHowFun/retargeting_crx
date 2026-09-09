"""Preview a closed bimanual EEF trajectory with offline IK; no hardware output."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from retargeting_apps.bimanual_trajectory import generate_arm_trajectory
from retargeting_apps.config import resolve_project_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/bimanual/crx5ia_coact_leap.yaml")
    parser.add_argument("--port", type=int, default=9220)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--samples", type=int, default=241)
    parser.add_argument("--amplitude", type=float, default=0.08, help="Reference joint amplitude (radians)")
    parser.add_argument("--headless", action="store_true", help="Validate IK and print errors without a viewer")
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")
    if args.samples < 3 or not np.isfinite(args.amplitude) or not 0 < args.amplitude <= 0.2:
        parser.error("--samples must be >= 3; --amplitude must be in (0, 0.2]")
    data = yaml.safe_load(resolve_project_path(args.config).read_text())
    trajectories = {}
    for side in ("left", "right"):
        trajectory = generate_arm_trajectory(data[side], args.samples, args.amplitude)
        trajectories[side] = trajectory
        print(f"{side}: max EEF error {1000 * trajectory['position_error_m'].max():.4f} mm, "
              f"{np.rad2deg(trajectory['rotation_error_rad'].max()):.4f} deg")
    if args.headless:
        return

    from retargeting_apps.visualization.execution.viser import _load_viser_dependencies
    from retargeting_apps.visualization.viser_scene import configure_initial_camera

    viser, ViserUrdf = _load_viser_dependencies()
    server = viser.ViserServer(port=args.port)
    try:
        configure_initial_camera(server, position=(1.1, 1.1, 0.9), look_at=(0, 0, 0.35))
        playing = server.gui.add_checkbox("Play", initial_value=True)
        frame = server.gui.add_slider("Frame", min=0, max=args.samples - 1, step=1, initial_value=0)
        server.gui.add_markdown("Yellow: target EEF; magenta: actual EEF. Kinematic IK preview.")
        handles = {}
        for side, trajectory in trajectories.items():
            placement = trajectory["placement"]
            server.scene.add_frame(
                f"/{side}_arm", show_axes=False, position=placement[:3, 3],
                wxyz=np.roll(Rotation.from_matrix(placement[:3, :3]).as_quat(), 1),
            )
            urdf = ViserUrdf(server, Path(trajectory["robot"].robot_file_path),
                             root_node_name=f"/{side}_arm", load_collision_meshes=False)
            points = trajectory["target"][:, :3, 3]
            server.scene.add_line_segments(
                f"/{side}_path", points=np.stack([points[:-1], points[1:]], axis=1),
                colors=(255, 220, 0), line_width=3,
            )
            markers = {}
            for kind, color in (("target", (255, 220, 0)), ("actual", (255, 0, 180))):
                markers[kind] = server.scene.add_frame(
                    f"/{side}_{kind}", axes_length=0.06 if kind == "target" else 0.04,
                    axes_radius=0.002, origin_color=color,
                )
            handles[side] = urdf, markers
        print(f"Open http://localhost:{args.port}; playback starts when a browser connects. Ctrl+C to stop.")
        while True:
            started = time.monotonic()
            index = int(frame.value)
            with server.atomic():
                for side, (urdf, markers) in handles.items():
                    trajectory = trajectories[side]
                    names = list(trajectory["robot"].actuated_joints)
                    q_by_name = dict(zip(names, trajectory["qpos"][index]))
                    urdf.update_cfg(np.asarray([
                        q_by_name[name] for name in urdf.get_actuated_joint_names()
                    ]))
                    for kind, marker in markers.items():
                        pose = trajectory[kind][index]
                        marker.position = pose[:3, 3]
                        marker.wxyz = np.roll(Rotation.from_matrix(pose[:3, :3]).as_quat(), 1)
            time.sleep(max(0, args.duration / (args.samples - 1) - (time.monotonic() - started)))
            if playing.value and server.get_clients():
                frame.value = (index + 1) % args.samples
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
