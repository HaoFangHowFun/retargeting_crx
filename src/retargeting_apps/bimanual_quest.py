"""Compatibility CLI for the unified bimanual execution application."""

import argparse

from retargeting.config.io import load_config_source
from retargeting_apps.apps.teleop_exe import run
from retargeting_apps.main import compose_hydra_base_config


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/bimanual/crx5ia_coact_leap.yaml")
    parser.add_argument("--port", type=int, default=9219)
    parser.add_argument("--adb", default=None)
    parser.add_argument("--serial", default=None)
    parser.add_argument("--backend", choices=("preview", "dual_crx"), default="preview")
    parser.add_argument("--left-hand-enabled", action="store_true")
    parser.add_argument("--right-hand-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--viewer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--command-hz", type=float, default=None,
                        help="Override command_hz from the selected backend YAML (default 20 Hz)")
    parser.add_argument("--teleoperation-mode", default="configs/teleoperation_modes/real_world.yaml",
                        help="Output-filter settings for physical commands")
    parser.add_argument("--duration", type=float, default=0.,
                        help="Stop seconds after tracking initializes; 0 means unlimited")
    args = parser.parse_args(argv)
    backend = "kinematic" if args.backend == "preview" else "dual_crx"
    config = compose_hydra_base_config([
        "app=teleop_exe", "teleoperation_modes=bimanual_quest", f"backends={backend}",
    ])
    setup = load_config_source(args.config)
    config["bimanual"].update(setup)
    config["bimanual"].update(
        duration=args.duration, left_hand_enabled=args.left_hand_enabled,
        right_hand_enabled=args.right_hand_enabled,
    )
    config["input"].update(load_config_source(setup["input"]["config"]))
    config["input"].update(
        port=setup["input"].get("port", 8765),
        max_age_s=setup["input"].get("max_age_s", 0.15), adb=args.adb, serial=args.serial,
    )
    if args.command_hz is not None:
        config["backend"]["command_hz"] = args.command_hz
    if args.backend == "dual_crx":
        config["teleoperation_mode"] = load_config_source(args.teleoperation_mode)
    config["viewer"].update(enabled=args.viewer, port=args.port, wait_for_client=False)
    run(config, [])


if __name__ == "__main__":
    main()
