"""Flat lifecycle owner for synchronized Quest solving and robot output."""
import time
import threading
import numpy as np

from teleoperation.bimanual import BimanualRetargetedFrame
from teleoperation.output import QposOutputFilter


class BimanualExecutionFlow:
    def __init__(self, *, source, pipeline, initial_qpos, backend_factory=None,
                 observer=None, command_hz=20., timeout=.25, duration=0.,
                 arm_output_filters: tuple[QposOutputFilter, QposOutputFilter] | None = None):
        if not np.isfinite(command_hz) or command_hz <= 0:
            raise ValueError("command_hz must be positive")
        if not np.isfinite(duration) or duration < 0:
            raise ValueError("duration must be finite and non-negative")
        self.duration = float(duration)
        self._duration_expired = threading.Event()
        self._duration_timer = None
        self.source, self.pipeline = source, pipeline
        self.initial_qpos = np.asarray(initial_qpos, dtype=float)
        if self.initial_qpos.shape != (44,) or not np.isfinite(self.initial_qpos).all():
            raise ValueError("expected 44 finite initial positions")
        if arm_output_filters is not None:
            if len(arm_output_filters) != 2:
                raise ValueError("arm_output_filters must contain left and right filters")
            if any(output_filter.previous_qpos.shape != (6,) for output_filter in arm_output_filters):
                raise ValueError("each arm output filter must contain six CRX joints")
        self.arm_output_filters = arm_output_filters
        self.backend_factory, self.observer = backend_factory, observer
        self.period, self.timeout = 1. / command_hz, timeout
        self.backend = None
        self.last_sequence = None
        self.last_command_at = None
        self.started_at = None
        self.tracking_paused = False
        self._recovery_started = None
        self._recovery_sequence = None
        self._last_qpos = self.initial_qpos.copy()

    def _reset_arm_output_filters(self, seed: np.ndarray) -> None:
        """Seed arm-only smoothing from the current measured robot pose."""
        if self.arm_output_filters is None:
            return
        left_filter, right_filter = self.arm_output_filters
        left_filter.reset(seed[:6])
        right_filter.reset(seed[22:28])

    def _filter_arm_commands(self, result: BimanualRetargetedFrame) -> BimanualRetargetedFrame:
        """Smooth only CRX J1-J6 while leaving both sixteen-joint hand targets raw."""
        if self.arm_output_filters is None:
            return result
        left_qpos = np.asarray(result.left_qpos, dtype=float).copy()
        right_qpos = np.asarray(result.right_qpos, dtype=float).copy()
        if left_qpos.shape != (22,) or right_qpos.shape != (22,):
            raise ValueError("bimanual filtering requires 22 positions per robot")
        left_filter, right_filter = self.arm_output_filters
        left_qpos[:6] = left_filter.apply(left_qpos[:6])
        right_qpos[:6] = right_filter.apply(right_qpos[:6])
        # Match the ordinary execution flow: the temporal objective follows the
        # filtered command actually visible to the robot, not the raw IK result.
        self.pipeline.left_retargeter.previous_qpos = left_qpos.copy()
        self.pipeline.right_retargeter.previous_qpos = right_qpos.copy()
        return BimanualRetargetedFrame(
            left_qpos=left_qpos,
            right_qpos=right_qpos,
            left_observation=result.left_observation,
            right_observation=result.right_observation,
        )

    def step(self, sample):
        """Send at most one command for each complete synchronized frame."""
        if self._duration_expired.is_set():
            return None
        if not sample.complete or sample.source_index is None:
            self._pause_tracking()
            return None
        if self.last_sequence is not None and sample.source_index <= self.last_sequence:
            return None
        if sample.left.source_index != sample.right.source_index:
            raise ValueError("left and right samples must come from the same Quest frame")
        if self.tracking_paused:
            now = time.monotonic()
            if self._recovery_sequence is not None and sample.source_index <= self._recovery_sequence:
                return None
            self._recovery_sequence = sample.source_index
            if self._recovery_started is None:
                self._recovery_started = now
            if now - self._recovery_started < .3:
                return None
            if self.backend is not None:
                self.backend.resume_tracking()
            if self._duration_expired.is_set():
                return None
            # Resume setup can block: calibrate using a NEW sample on the next tick.
            self.pipeline.reset()
            self.tracking_paused = False
            self._recovery_started = None
            self.last_sequence = sample.source_index
            self.last_command_at = time.monotonic()
            print("Quest tracking recovered; recalibrating from current robot pose.", flush=True)
            return None
        if not self.pipeline.initialized:
            if self.backend is None and self.backend_factory is not None:
                self.backend = self.backend_factory()
                # Startup may take seconds. Acquire a new frame before calibration.
                return None
            seed = self._last_qpos if self.backend is None else self.backend.get_joint_pos()
            self.pipeline.left_retargeter.reset(seed[:22])
            self.pipeline.right_retargeter.reset(seed[22:])
            self._reset_arm_output_filters(seed)
            if not self.pipeline.initialize(sample, seed[:22], seed[22:]):
                return None
            first_start = self.started_at is None
            if first_start:
                self.started_at = time.monotonic()
            if first_start and self.duration > 0:
                self._duration_timer = threading.Timer(self.duration, self._expire_duration)
                self._duration_timer.daemon = True
                self._duration_timer.start()
                print(f"Quest tracking initialized; automatic stop in {self.duration:g} seconds.", flush=True)
        started = time.monotonic()
        result = self.pipeline.step(sample)
        self.last_sequence = sample.source_index
        if result is None or self._duration_expired.is_set():
            return None
        # Never refresh an obsolete input into a new ROS command after a slow solve.
        received_ns = getattr(getattr(sample.left, "raw", None), "received_monotonic_ns", None)
        age = time.monotonic() - started if received_ns is None else (time.monotonic_ns() - received_ns) / 1e9
        if age > self.source.max_age_s:
            return None
        result = self._filter_arm_commands(result)
        if self.backend is not None:
            try:
                self.backend.execute(result.qpos)
            except RuntimeError:
                if self._duration_expired.is_set():
                    return None
                raise
        self._last_qpos = result.qpos.copy()
        self.last_command_at = time.monotonic()
        if self.observer is not None:
            self.observer(result)
        return result

    def _pause_tracking(self):
        self._recovery_started = None
        self._recovery_sequence = None
        if self.started_at is None or self.tracking_paused:
            return
        if self.backend is not None:
            self.backend.pause_tracking()
        self.tracking_paused = True
        print("Quest tracking lost; holding robots, waiting for both hands.", flush=True)

    def _expire_duration(self):
        # Independent of the solver loop: request stop even if a solve is slow.
        self._duration_expired.set()
        if self.backend is not None:
            self.backend.request_stop("Quest session duration reached")

    def run(self):
        try:
            self.source.open()
            next_command = time.monotonic()
            last_report = 0.
            while True:
                now = time.monotonic()
                if self._duration_expired.is_set():
                    print("Quest session duration reached; stopping teleoperation.", flush=True)
                    return
                if self.backend is not None and self.started_at is not None:
                    if now - self.started_at > .5:
                        self.backend.assert_tracking()
                    reference = self.last_command_at if self.last_command_at is not None else self.started_at
                    if not self.tracking_paused and now - reference > self.timeout:
                        self._pause_tracking()
                if now >= next_command:
                    self.step(self.source.read())
                    next_command = max(now + self.period, time.monotonic())
                if now - last_report > 2.:
                    stats = self.source.stats
                    print(f"Quest frames={getattr(stats, 'accepted', 0)} "
                          f"initialized={self.pipeline.initialized} sequence={self.last_sequence}", flush=True)
                    last_report = now
                time.sleep(.002)
        finally:
            if self._duration_timer is not None:
                self._duration_timer.cancel()
                self._duration_timer.join()
            try:
                if self.backend is not None:
                    self.backend.close()
            finally:
                self.source.close()
