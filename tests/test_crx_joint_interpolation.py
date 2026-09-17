"""Pure interpolation and publisher lifecycle checks; no ROS or hardware."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from retargeting_ros.joint_interpolation import JointCommandInterpolator

# Reuse the existing transport fixture, not another implementation of its state.
from test_crx_joint_script import connection, script, state


@pytest.mark.parametrize('method', ['linear', 'cubic'])
def test_initial_transition_endpoint_hold_and_late_tick(method):
    initial = np.arange(12)*.1
    curve = JointCommandInterpolator(initial, method)
    np.testing.assert_array_equal(curve.sample(1.), initial)
    curve.set_target(initial+.1, 1., .05, 1.01)
    np.testing.assert_array_equal(curve.sample(1.01), initial)
    np.testing.assert_allclose(curve.sample(1.03), initial+.05)
    np.testing.assert_array_equal(curve.sample(1.05), initial+.1)
    np.testing.assert_array_equal(curve.sample(50.), initial+.1)


def test_cubic_uses_published_history_and_duplicate_stamps_are_replaced():
    curve = JointCommandInterpolator(np.zeros(12))
    for t in (1., 1.01, 1.02, 1.03, 1.04):
        curve.record_published(t, np.full(12, t-1))
    curve.record_published(1.04, np.full(12, .04))
    assert len(curve.history) == 5
    curve.set_target(np.full(12, .09), 1.04, .05, 1.045)
    assert curve.segment[-1] is not None
    np.testing.assert_allclose(curve.sample(1.065), .065, atol=1e-12)
    np.testing.assert_array_equal(curve.sample(1.1), np.full(12, .09))


@pytest.mark.parametrize('interval', [.05, .052, .06, .08])
def test_irregular_targets_keep_finite_samples_and_exact_final_hold(interval):
    curve = JointCommandInterpolator(np.zeros(12))
    next_target = .1
    count = 0
    last = np.zeros(12)
    for t in np.arange(.1, 2., .01):
        if t+1e-10 >= next_target:
            last = np.full(12, .01*np.sin(2*np.pi*t))
            curve.set_target(last, t, .05, t)
            next_target += interval
        q = curve.sample(t)
        assert np.isfinite(q).all()
        curve.record_published(t, q)
        count += 1
    assert count == 190
    np.testing.assert_array_equal(curve.sample(3.), last)


def test_natural_cubic_overshoot_is_exposed_not_silently_clipped():
    curve = JointCommandInterpolator(np.zeros(12))
    for t in (1., 1.01, 1.02, 1.03, 1.04):
        curve.record_published(t, np.full(12, 2*(t-1.04)))
    curve.set_target(np.zeros(12), 1.04, .05, 1.04)
    assert max(curve.sample(t)[0] for t in np.linspace(1.04, 1.09, 101)) > .01
    np.testing.assert_array_equal(curve.sample(1.1), np.zeros(12))


def test_invalid_targets_times_and_failed_fit_preserve_previous_segment():
    curve = JointCommandInterpolator(np.zeros(12))
    curve.set_target(np.ones(12), 1., .05, 1.)
    segment = curve.segment
    for target, received, horizon, now in [
        ([0]*11, 1., .05, 1.), ([np.nan]*12, 1., .05, 1.),
        ([0]*12, 1., 0., 1.), ([0]*12, 1., .05, .9),
        ([0]*12, 1., .05, 1.1), ([0]*12, np.inf, .05, np.inf),
    ]:
        with pytest.raises(ValueError):
            curve.set_target(target, received, horizon, now)
        assert curve.segment is segment
    curve.record_published(1., np.zeros(12))
    with pytest.raises(ValueError):
        curve.record_published(.9, np.zeros(12))


@pytest.fixture
def timed_connection(connection, monkeypatch):
    link = connection
    now = [10.]
    monkeypatch.setattr(script, 'time', NS(monotonic=lambda: now[0]))
    link._publish_hz = 100.
    timer = NS(canceled=True, resets=0)
    timer.cancel = lambda: setattr(timer, 'canceled', True)

    def reset():
        timer.canceled = False
        timer.resets += 1

    timer.reset = reset
    timer.is_canceled = lambda: timer.canceled
    link._output_timer = timer
    link._receive_state(state(positions=[.1]*12))
    link._reset_output_locked(link._actual[script.ARM_INDICES])
    return link, now


def test_execute_submits_latest_target_timer_starts_once_and_holds(timed_connection):
    link, now = timed_connection
    target = np.full(44, .2)
    link._publish_tick()
    assert not link.commands
    link.execute(target)
    link.execute(target+.1)
    assert not link.commands and link._output_timer.resets == 1
    np.testing.assert_array_equal(link.get_target_joint_pos(), target+.1)
    for t in (10.01, 10.02, 10.04, 10.06, 10.08):
        now[0] = t
        link._publish_tick()
    assert len(link.commands) == 5
    np.testing.assert_array_equal(link.commands[0], [.1]*12)
    np.testing.assert_allclose(link.commands[-1], [.3]*12)
    assert link._target_count == 2 and link._publish_count == 5


@pytest.mark.parametrize('action', ['pause_tracking', 'request_stop', 'reset'])
def test_lifecycle_cancels_output_and_discards_pending(timed_connection, action):
    link, now = timed_connection
    link.execute(np.full(44, .2))
    now[0] += .01
    link._publish_tick()
    getattr(link, action)(*(['test'] if action == 'request_stop' else []))
    assert link._output_timer.canceled and link._pending_target is None
    now[0] += .01
    link._publish_tick()
    assert len(link.commands) == 1
    if action == 'pause_tracking':
        link._receive_state(state(positions=[.4]*12, nanosec=1))
        link.resume_tracking()
        assert not link._interpolator.history
        link.execute(np.full(44, .5))
        now[0] += .01
        link._publish_tick()
        np.testing.assert_array_equal(link.commands[-1], [.4]*12)


def test_input_expiry_is_not_refreshed_by_repeated_publications(timed_connection):
    link, now = timed_connection
    link.execute(np.full(44, .2))
    for t in np.arange(10.01, 10.3, .01):
        now[0] = t
        link._receive_state(state(positions=[.1]*12))
        link._publish_tick()
    assert link._output_timer.canceled and link._last_target_at is None
    count = len(link.commands)
    link._publish_tick()
    assert len(link.commands) == count
    link.execute(np.full(44, .3))
    assert not link._output_timer.canceled


def test_bad_feedback_stops_the_background_publisher(timed_connection):
    link, now = timed_connection
    link.execute(np.full(44, .2))
    link._receive_state(state(positions=[np.nan]*12))
    link._publish_tick()
    assert link._stopped and link._output_timer.canceled
    assert not link.commands


def test_expired_target_is_rejected_without_jumping(timed_connection):
    link, now = timed_connection
    link.execute(np.full(44, .2))
    now[0] += .06
    link._publish_tick()
    assert not link.commands and link._rejected_segments == 1


def test_stop_during_curve_calculation_cannot_publish(timed_connection, monkeypatch):
    link, now = timed_connection
    link.execute(np.full(44, .2))
    now[0] += .01
    interpolator = link._interpolator
    sample = interpolator.sample

    def stop_and_sample(t):
        link.request_stop('duration expired during fit')
        return sample(t)

    monkeypatch.setattr(interpolator, 'sample', stop_and_sample)
    link._publish_tick()
    assert not link.commands


def test_long_curve_calculation_cannot_publish_an_obsolete_sample(timed_connection, monkeypatch):
    link, now = timed_connection
    link.execute(np.full(44, .2))
    now[0] += .01
    interpolator = link._interpolator
    sample = interpolator.sample

    def slow_sample(t):
        now[0] += .02
        return sample(t)

    monkeypatch.setattr(interpolator, 'sample', slow_sample)
    link._publish_tick()
    assert not link.commands and link._rejected_segments == 1


def test_invalid_horizon_is_rejected_before_opening_source():
    args = NS(command_hz=20., duration=1., serial=None, viewer=False, viewer_port=9219,
              publish_hz=100., output_interpolation='cubic', interpolation_horizon_ms=500.)
    with pytest.raises(ValueError, match='shorter than'):
        script.build_flow(args)
