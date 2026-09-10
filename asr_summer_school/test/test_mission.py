"""Decision and controller regression tests; no robot movement or DDS required."""
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from asr_summer_school.mission_logic import (
    MissionClock, atomic_yaml, clear_point, path_is_known, summarize,
)
from asr_summer_school.example_nav_to_pose import Mission


def grid():
    return NS(info=NS(width=10, height=10, resolution=0.1,
                     origin=NS(position=NS(x=0., y=0.),
                               orientation=NS(x=0., y=0., z=0., w=1.))), data=[0]*100)


def pose(x, y):
    return NS(pose=NS(position=NS(x=x, y=y)))


def test_optional_stopwatch_clock():
    clock = MissionClock()
    clock.started = 50
    assert clock.remaining(100000) is None
    assert clock.fits(100000, 100000)
    assert clock.elapsed(60) == 10


@pytest.mark.parametrize('value', [-1, float('nan'), float('inf')])
def test_invalid_duration(value):
    with pytest.raises(ValueError):
        MissionClock(value)


def test_deadline_reserve():
    clock = MissionClock(60)
    clock.started = 100
    assert clock.fits(120, 30)
    assert not clock.fits(130, 30)
    assert not clock.fits(161, 0)


def test_unknown_and_obstacle_clearance():
    g = grid()
    assert clear_point(g, .5, .5, .1)
    g.data[55] = -1
    assert not clear_point(g, .5, .5, .1)
    g.data[55] = 100
    assert not clear_point(g, .5, .5, .1)
    assert not clear_point(g, 0., 0., .1)


def test_sparse_path_cannot_jump_wall():
    g = grid()
    path = NS(poses=[pose(.2, .5), pose(.8, .5)])
    assert path_is_known(g, path, .01)
    for row in range(10):
        g.data[row*10+5] = 100
    assert not path_is_known(g, path, .01)
    assert not path_is_known(g, NS(poses=[]), .01)


def test_rotated_map_origin():
    g = grid()
    g.info.origin.position.x = 1.
    g.info.origin.orientation.z = math.sin(math.pi/4)
    g.info.origin.orientation.w = math.cos(math.pi/4)
    assert clear_point(g, .5, .5, .1)


def test_confirmation_and_outlier():
    samples = [{'position': p} for p in ([1., 2., .2], [1.01, 2., .2], [1., 2.02, .2], [9., 9., 9.])]
    result = summarize(samples, .3)
    assert result['confirmed']
    assert result['consistent_observations'] == 3
    assert result['position']['x'] < 1.02
    assert not summarize(samples[:1], .3)['confirmed']


def test_atomic_export(tmp_path):
    import yaml
    path = tmp_path/'run'/'semantic.yaml'
    atomic_yaml(path, {'tags': []})
    atomic_yaml(path, {'tags': [3]})
    assert yaml.safe_load(path.read_text()) == {'tags': [3]}
    assert not path.with_suffix('.yaml.tmp').exists()


def make_mission(duration=0.):
    defaults = dict(mission_duration_sec=duration, return_reserve_sec=30., return_speed_mps=.1,
                    goal_timeout_sec=120., return_timeout_sec=180., failed_cooldown_sec=60.,
                    save_interval_sec=30., footprint_radius_m=.15, sensor_timeout_sec=3.,
                    observe_sec=15., output_dir='/tmp/mission_test', speed_profiles_enabled=False,
                    turn_speed_rps=.4, planning_allowance_sec=10., recovery_allowance_sec=20.,
                    late_return_timeout_sec=120., selection_timeout_sec=5.)
    sensors = Mock()
    sensors.odom = None
    sensors.param.side_effect = defaults.__getitem__
    sensors.return_requested.is_set.return_value = False
    m = Mission(Mock(), sensors)
    m.clock.started = 0.
    return m


def test_untimed_manual_return(monkeypatch):
    m = make_mission()
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.monotonic', lambda: 5.)
    m.last_home_check, m.home_seconds = 5., 100.
    m.home_plan_stamp = 5.
    m.budget = {'total_sec': 130.}
    assert not m.check_return()
    m.sensors.return_requested.is_set.return_value = True
    assert m.check_return()
    assert m.reason == 'Operator requested return'


def test_no_home_path_stops_exploration(monkeypatch):
    m = make_mission()
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.monotonic', lambda: 5.)
    m.last_home_check = 5.
    m.plan = Mock(return_value=None)
    m.pose = Mock()
    assert m.check_return()


def test_cancellation_waits_for_terminal_result(monkeypatch):
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.rclpy.ok', lambda: True)
    m = make_mission()
    m.active = True
    m.navigator.isTaskComplete.side_effect = [False, True]
    m.cancel()
    m.navigator.goal_handle.cancel_goal_async.assert_called_once()
    assert not m.active


def test_final_export_reports_failure(tmp_path):
    m = make_mission()
    m.output = tmp_path
    m.sensors.save_tags.service_is_ready.return_value = False
    m.sensors.save_map.service_is_ready.return_value = False
    m.sensors.snapshot.return_value = (None, None, 0, {}, None, {'unique': 0})
    m.save()
    assert not m.map_saved and not m.tags_saved
    assert (tmp_path/'mission_result.yaml').exists()


def test_return_budget_turns_and_duplicates():
    from asr_summer_school.mission_logic import return_budget
    path = NS(poses=[pose(0, 0), pose(0, 0), pose(1, 0), pose(1, 1)])
    budget = return_budget(path)
    assert budget['travel_sec'] == 20.
    assert budget['turning_sec'] == pytest.approx(math.pi/2/.4)
    assert budget['total_sec'] == pytest.approx(80 + math.pi/2/.4)


def test_stale_home_plan_latches_return(monkeypatch):
    m = make_mission()
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.monotonic', lambda: 20.)
    m.home_plan_stamp = 1.
    assert m.check_return()
    m.home_plan_stamp = 20.
    assert m.check_return()
    assert m.reason == 'Home plan stale'


def test_late_return_shared_deadline(monkeypatch):
    m = make_mission(60.)
    m.return_started = 40.
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.monotonic', lambda: 61.)
    assert not m.return_expired()
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.monotonic', lambda: 180.)
    assert m.return_expired()
    m.return_started = 179.  # Retrying must not reset the absolute limit.
    assert m.return_expired()


def test_deadline_result_independent_of_export():
    m = make_mission(60.)
    m.arrival = 60.
    m.export_completed = 80.
    assert m.result_details()['returned_before_deadline']
    m.arrival = 60.01
    assert not m.result_details()['returned_before_deadline']
    m.clock.duration = 0.
    assert m.result_details()['returned_before_deadline'] is None


def test_expired_selection_does_not_submit(monkeypatch):
    m = make_mission()
    m.selection_deadline = 1.
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.monotonic', lambda: 2.)
    assert m.plan(None, None) is None
    m.sensors.planner.send_goal_async.assert_not_called()


def test_export_failure_does_not_skip_map(tmp_path):
    m = make_mission()
    m.output = tmp_path
    m.sensors.save_tags.call_async.side_effect = RuntimeError('tag failure')
    m.export_wait = Mock(side_effect=[None, NS(result=True)])
    m.sensors.snapshot.return_value = (None, None, 0, {}, None, {})
    m.save()
    assert not m.tags_saved and m.map_saved
    assert m.export_wait.call_count == 2


@pytest.mark.parametrize('speed,expected', [(0., True), (.05, False)])
def test_home_requires_stationary_odometry(monkeypatch, speed, expected):
    m = make_mission()
    now = [1.]
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.monotonic', lambda: now[0])
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.sleep', lambda t: now.__setitem__(0, now[0]+t))
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.rclpy.ok', lambda: True)
    m.healthy = Mock(return_value=True)
    m.home = pose(0., 0.)
    m.pose = Mock(return_value=pose(0., 0.))
    m.sensors.odom = NS(twist=NS(twist=NS(linear=NS(x=speed, y=0.), angular=NS(z=0.))))
    assert m.verify_home() is expected
    assert (m.arrival is not None) is expected


def test_failed_background_plan_returns(monkeypatch):
    from concurrent.futures import Future
    m = make_mission()
    m.home_plan_stamp = 1.
    m.home_future = Future()
    m.home_future.set_exception(RuntimeError('planner unavailable'))
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.monotonic', lambda: 2.)
    assert m.check_return()


def test_cached_route_accounts_for_motion_while_refresh_pending(monkeypatch):
    from concurrent.futures import Future
    m = make_mission(100.)
    m.home_plan_stamp = 1.
    m.budget = {'total_sec': 60.}
    m.home_plan_distance = 0.
    m.odom_distance = 2.
    m.home_future = Future()
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.monotonic', lambda: 5.)
    assert not m.check_return()
    assert m.home_seconds == 50.  # 60 budget + 20 travel - 30 margin


def test_cancellation_failure_still_exports():
    m = make_mission()
    m.cancel = Mock(side_effect=RuntimeError('Cancellation refused'))
    m.export_wait = Mock()
    m.save = Mock()
    m.save_result = Mock()
    m.failure_cleanup()
    m.export_wait.assert_called_once()
    m.save.assert_called_once()
    m.save_result.assert_called_once()
    assert 'Cancellation refused' in m.cleanup_errors


@pytest.mark.parametrize('arrival,expected', [(59., 'FINISHED'), (61., 'FAILED')])
def test_full_return_flow_records_lateness(monkeypatch, tmp_path, arrival, expected):
    m = make_mission(60.)
    defaults = m.sensors.param.side_effect.__self__
    defaults['challenge_mode'] = False
    m.sensors.start_time = 0.
    m.sensors.start_requested.is_set.return_value = True
    m.output = tmp_path
    m.home = pose(0., 0.)
    m.home.header = NS(frame_id='map')
    m.home.pose.orientation = NS(x=0., y=0., z=0., w=1.)
    m.pose = Mock(return_value=m.home)
    m.healthy = Mock(return_value=True)
    m.check_return = Mock(return_value=True)
    m.wait = Mock(return_value=NS(success=True))
    m.export_wait = Mock(return_value=NS(success=True))
    m.publish = Mock()
    m.plan = Mock(return_value=True)
    m.navigate = Mock(return_value=True)
    calls = [False, True]
    def verify():
        success = calls.pop(0)
        if success:
            m.returned, m.arrival = True, arrival
        return success
    m.verify_home = verify
    def save():
        m.map_saved = m.tags_saved = True
    m.save = save
    m.sensors.snapshot.return_value = (None, None, 0, {}, None, {})
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.rclpy.ok', lambda: True)
    monkeypatch.setattr('asr_summer_school.example_nav_to_pose.time.monotonic', lambda: arrival)
    m.run()
    assert m.state == expected
    m.navigate.assert_called_once_with(m.home, returning=True)
    import yaml
    result = yaml.safe_load((tmp_path/'mission_result.yaml').read_text())
    assert result['returned_home']
    assert result['returned_before_deadline'] == (arrival <= 60.)
