"""Run the real CRX+LEAP kinematic flow on archived AVP and save diagnostics."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from retargeting_apps.composition import build_execution_flow
from retargeting_apps.main import compose_hydra_base_config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--output", type=Path, default=Path("outputs/crx5ia_leap_offline"))
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    config = compose_hydra_base_config([
        "app=teleop_exe",
        "retargeting_profiles=vector_wrist_joint_crx5ia_leap_paxini",
        "teleoperation_modes=offline_kinematic",
        "input.data=tests/fixtures/avp_teleop_2025-01-16_20-27-43.npz",
        f"input.end={args.frames - 1}",
        "evaluate=true",
    ])
    flow = build_execution_flow(config)
    records = []

    def record(result):
        frame = result.retargeted_frame
        if frame is None:
            return
        adaptor = flow.retargeter.robot_adaptor
        model = adaptor.robot_model
        target = frame.observation.wrist_pose_world
        actual = model.get_frame_pose("wrist", adaptor.forward_qpos(result.actual_qpos)).copy()
        records.append({
            "raw_qpos": frame.retargeted_qpos,
            "command_qpos": result.command_qpos,
            "actual_qpos": result.actual_qpos,
            "target_wrist": target,
            "actual_wrist": actual,
            "position_error_m": np.linalg.norm(target[:3, 3] - actual[:3, 3]),
            "orientation_error_deg": np.rad2deg(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude()),
            "solve_time_s": frame.diagnostics["optimization_time"],
        })

    flow.add_step_observer(record)
    summary = flow.run()
    if not records:
        raise RuntimeError("No valid hand frames retargeted")
    args.output.mkdir(parents=True, exist_ok=True)
    arrays = {key: np.asarray([record[key] for record in records]) for key in records[0]}
    np.savez_compressed(args.output / "diagnostics.npz", **arrays)
    metadata = {
        "profile": "vector_wrist_joint_crx5ia_leap_paxini",
        "input": "tests/fixtures/avp_teleop_2025-01-16_20-27-43.npz",
        "input_kind": "archived AVP, not recorded Quest or physical CRX",
        "backend": "kinematic",
        "actuated_joints": list(flow.retargeter.robot_config.actuated_joints),
        "initial_qpos": list(flow.retargeter.robot_config.initial_qpos),
        "frames": len(records),
        "source_frames": summary.source_frames_processed,
        "max_position_error_m": float(arrays["position_error_m"].max()),
        "max_orientation_error_deg": float(arrays["orientation_error_deg"].max()),
        "mean_solve_time_s": float(arrays["solve_time_s"].mean()),
        "interpretation": "Soft hand-vector objective; residuals are not strict wrist IK success criteria. No collision or hardware validation.",
    }
    (args.output / "metadata.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
    print(yaml.safe_dump(metadata, sort_keys=False))


if __name__ == "__main__":
    main()
