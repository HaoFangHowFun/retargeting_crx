"""Check live Sharpa scale controls without opening Quest, ROS, or Viser."""

from types import SimpleNamespace as NS

import numpy as np

from scripts import preview_sharpa_scale
from scripts.preview_sharpa_scale import add_scale_controls
from retargeting_apps.sharpa_teleop import build_flow
from retargeting_apps.visualization.execution import manager
from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput


class Slider:
    def __init__(self, value):
        self.value = value

    def on_update(self, callback):
        self.callback = callback

    def change(self, value):
        self.value = value
        self.callback(NS(target=self))


class Gui:
    def add_slider(self, label, **options):
        return Slider(options["initial_value"])


def test_slider_changes_only_its_hand_keypoints():
    args = NS(config=None, backend="preview", duration=0.0, command_hz=20.0,
              adb=None, serial=None, viewer=True, viewer_port=9219)
    source = SyntheticBimanualInput()
    flow, _ = build_flow(args, with_arms=False, source=source)
    controls = add_scale_controls(flow, NS(server=NS(gui=Gui())),
                                  left_scale=1.0, right_scale=1.0)
    sample = source.read()
    left, right = flow.pipeline.left_mapper, flow.pipeline.right_mapper
    assert left.initialize(sample.left, flow.initial_qpos[:22])
    assert right.initialize(sample.right, flow.initial_qpos[22:])
    left_before = left.map(sample.left).keypoints_wrist
    right_before = right.map(sample.right).keypoints_wrist

    controls["left"].change(1.5)

    np.testing.assert_allclose(left.map(sample.left).keypoints_wrist, left_before * 1.5)
    np.testing.assert_array_equal(right.map(sample.right).keypoints_wrist, right_before)
    controls["right"].change(0.8)
    np.testing.assert_allclose(right.map(sample.right).keypoints_wrist, right_before * 0.8)
    np.testing.assert_allclose(left.map(sample.left).keypoints_wrist, left_before * 1.5)
    assert flow.backend_factory is None


def test_demo_passes_synthetic_source_without_opening_quest(monkeypatch):
    captured = {}
    pipeline = NS(left_mapper=NS(human_hand_scale=1.0),
                  right_mapper=NS(human_hand_scale=1.0))
    flow = NS(pipeline=pipeline, hand_output_filters=(object(), object()),
              run=lambda: None)

    def fake_build_flow(args, *, with_arms, source):
        captured.update(source=source, backend=args.backend, with_arms=with_arms)
        return flow, {"viewer": {"human_keypoint_size": 0.006}}

    monkeypatch.setattr("retargeting_apps.sharpa_teleop.build_flow", fake_build_flow)

    def fake_visualizer(config, flow):
        captured["keypoint_size"] = config["viewer"]["human_keypoint_size"]
        return NS(server=NS(gui=Gui()), close=lambda: None)

    monkeypatch.setattr(manager, "create_optional_execution_visualizer", fake_visualizer)

    preview_sharpa_scale.main(["--demo"])

    assert isinstance(captured["source"], SyntheticBimanualInput)
    assert captured["backend"] == "preview"
    assert captured["with_arms"] is False
    assert captured["keypoint_size"] == 0.006
    assert len(flow.hand_output_filters) == 2
