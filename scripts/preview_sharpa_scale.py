#!/usr/bin/env python3
"""Compare Quest hand tracking with Sharpa Wave using live scale sliders."""

import argparse
import math
import sys


def add_scale_controls(flow, visualizer, *, left_scale=None, right_scale=None):
    """Apply scale changes to the next mapped Quest frame without ROS output."""
    controls = {}
    for side, requested in (("left", left_scale), ("right", right_scale)):
        mapper = getattr(flow.pipeline, f"{side}_mapper")
        scale = mapper.human_hand_scale if requested is None else requested
        if not math.isfinite(scale) or not 0.5 <= scale <= 2.0:
            raise ValueError(f"{side} scale must be between 0.5 and 2.0")
        mapper.human_hand_scale = scale
        slider = visualizer.server.gui.add_slider(
            f"{side.capitalize()} Quest hand scale", min=0.5, max=2.0,
            step=0.01, initial_value=scale,
            hint="Scales Quest wrist-local hand keypoints; does not resize the Sharpa URDF.",
        )

        def update(event, *, target=mapper):
            target.human_hand_scale = float(event.target.value)

        slider.on_update(update)
        controls[side] = slider
    return controls


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-scale", type=float, default=None,
                        help="Initial left scale (default: Sharpa robot config)")
    parser.add_argument("--right-scale", type=float, default=None,
                        help="Initial right scale (default: Sharpa robot config)")
    parser.add_argument("--command-hz", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Seconds after calibration; 0 runs until Ctrl+C")
    parser.add_argument("--serial", default=None, help="ADB headset serial")
    parser.add_argument("--adb", default=None, help="ADB executable")
    parser.add_argument("--viewer-port", type=int, default=9219)
    parser.add_argument("--demo", action="store_true",
                        help="Use animated synthetic hands without Quest or ROS")
    args = parser.parse_args(argv)
    if not math.isfinite(args.command_hz) or args.command_hz <= 0:
        parser.error("--command-hz must be finite and positive")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and nonnegative")
    if not 1 <= args.viewer_port <= 65535:
        parser.error("--viewer-port must be between 1 and 65535")
    for side in ("left", "right"):
        scale = getattr(args, f"{side}_scale")
        if scale is not None and (not math.isfinite(scale) or not 0.5 <= scale <= 2.0):
            parser.error(f"--{side}-scale must be between 0.5 and 2.0")

    from retargeting_apps.sharpa_teleop import build_flow
    from retargeting_apps.visualization.execution.manager import create_optional_execution_visualizer

    args.backend = "preview"
    args.config = None
    args.viewer = True
    source = None
    if args.demo:
        from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput

        source = SyntheticBimanualInput(frames=sys.maxsize)
        print("Synthetic demo selected; its hand size does not calibrate Quest tracking.", flush=True)
    flow, config = build_flow(args, with_arms=False, source=source)
    visualizer = create_optional_execution_visualizer(config, flow)
    try:
        add_scale_controls(flow, visualizer, left_scale=args.left_scale,
                           right_scale=args.right_scale)
        print("Quest vs Sharpa preview: adjust left/right scale sliders in the viewer.", flush=True)
        try:
            flow.run()
        except KeyboardInterrupt:
            pass
    finally:
        print("Final human_hand_scale: "
              f"left={flow.pipeline.left_mapper.human_hand_scale:.2f}, "
              f"right={flow.pipeline.right_mapper.human_hand_scale:.2f}", flush=True)
        visualizer.close()


if __name__ == "__main__":
    main()
