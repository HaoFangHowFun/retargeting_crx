"""Finite synthetic bilateral hand input for explicit hardware-free smoke tests."""

import time
from types import SimpleNamespace

import numpy as np

from teleoperation.types import BimanualSensorHandSample, SensorHandSample


class SyntheticBimanualInput:
    def __init__(self, frames=100, max_age_s=.15):
        if isinstance(frames, bool) or not isinstance(frames, int) or frames <= 0:
            raise ValueError('frames must be a positive integer')
        self.frames, self.max_age_s = frames, max_age_s
        self.stats = SimpleNamespace(accepted=0)

    def open(self):
        self.stats.accepted = 0

    def read(self):
        if self.stats.accepted >= self.frames:
            raise StopIteration
        index = self.stats.accepted
        self.stats.accepted += 1
        hands = []
        for side in ('left', 'right'):
            points = np.zeros((21, 3))
            curl = .2 * (1 - np.cos(index * .12))
            for finger in range(5):
                lateral = (.05, .03, .01, -.01, -.03)[finger]
                for segment in range(4):
                    distance = .025 * (segment + 1)
                    # Match decoded Quest: +Z along fingers, flexion toward +X.
                    points[1 + 4 * finger + segment] = (
                        distance * np.sin(curl * segment),
                        lateral * (1 if side == 'right' else -1),
                        .035 + distance * np.cos(curl * segment),
                    )
            hands.append(SensorHandSample(
                keypoints_wrist=points, wrist_pose_sensor=np.eye(4), timestamp=time.monotonic(),
                raw=SimpleNamespace(received_monotonic_ns=time.monotonic_ns()), source_index=index,
            ))
        return BimanualSensorHandSample(left=hands[0], right=hands[1])

    def close(self):
        pass
