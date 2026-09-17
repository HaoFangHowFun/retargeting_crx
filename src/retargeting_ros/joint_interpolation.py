"""ROS-independent, time-sampled interpolation of twelve CRX joint commands.

Natural cubic follows RobotRealHighFreq's published-history approach. It does
not impose joint, velocity or acceleration limits and may overshoot.
"""
from collections import deque

import numpy as np
from scipy.interpolate import CubicSpline


def joint_vector(values):
    q = np.asarray(values, dtype=float)
    if q.shape != (12,) or not np.isfinite(q).all():
        raise ValueError('Expected 12 finite joint positions')
    return q.copy()


class JointCommandInterpolator:
    """Build on the last published sample; evaluate using monotonic time."""

    def __init__(self, initial, method='cubic'):
        if method not in ('linear', 'cubic'):
            raise ValueError('Interpolation method must be linear or cubic')
        self.method = method
        self.last_q = joint_vector(initial)
        self.last_time = None
        self.history = deque(maxlen=5)
        self.segment = None

    def set_target(self, target, received_at, horizon, now):
        q = joint_vector(target)
        if not np.isfinite([received_at, horizon, now]).all() or horizon <= 0 or now < received_at:
            raise ValueError('Invalid interpolation times or horizon')
        start = now if self.last_time is None else self.last_time
        end = received_at + horizon
        if now >= end or start > now:
            raise ValueError('Target deadline expired or monotonic time moved backwards')
        spline = None
        if self.method == 'cubic' and len(self.history) >= 2:
            points = [(t, value) for t, value in self.history if t < start]
            points.extend([(start, self.last_q), (end, q)])
            times = np.array([t for t, _ in points]) - start
            spline = CubicSpline(times, np.array([value for _, value in points]),
                                 axis=0, bc_type='natural', extrapolate=False)
        # Commit only after the complete new segment is valid.
        self.segment = (start, end, self.last_q.copy(), q, spline)

    def sample(self, now):
        if not np.isfinite(now) or (self.last_time is not None and now < self.last_time):
            raise ValueError('Sample time must be finite and monotonic')
        if self.segment is None:
            return self.last_q.copy()
        start, end, initial, target, spline = self.segment
        if now >= end:
            return target.copy()
        if now <= start:
            return initial.copy()
        if spline is not None:
            return joint_vector(spline(now - start))
        phase = (now - start) / (end - start)
        return joint_vector(initial + phase * (target - initial))

    def record_published(self, now, q):
        values = joint_vector(q)
        if not np.isfinite(now) or (self.last_time is not None and now < self.last_time):
            raise ValueError('Publish time must be finite and monotonic')
        if self.last_time == now and self.history:
            self.history.pop()
        self.last_q, self.last_time = values, now
        self.history.append((now, values.copy()))
