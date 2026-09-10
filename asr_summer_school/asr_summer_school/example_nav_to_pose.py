#!/usr/bin/env python3
"""Onboard search-and-rescue mission, extending the original go-to-pose demo."""
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav2_msgs.srv import SaveMap
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from asr_summer_school.sensor_monitor import MissionSensors
from asr_summer_school.mission_logic import (
    MissionClock, atomic_yaml, clear_point, path_is_known, path_length, return_budget,
)


class BoundedNavigator(BasicNavigator):
    """Keep the example API while bounding goal-acceptance waits on hardware."""

    def _send(self, client, request):
        if not client.server_is_ready():
            raise RuntimeError('Navigation action server unavailable')
        future = client.send_goal_async(request, feedback_callback=self._feedbackCallback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=getattr(self, 'acceptance_timeout', 3.0))
        if not future.done():
            def cancel_late(done):
                handle = done.result()
                if handle and handle.accepted:
                    handle.cancel_goal_async()
            future.add_done_callback(cancel_late)
            raise RuntimeError('Navigation goal acceptance timed out')
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            return False
        self.feedback = None
        self.result_future = self.goal_handle.get_result_async()
        return True

    def goToPose(self, pose, behavior_tree=''):
        goal = NavigateToPose.Goal()
        goal.pose, goal.behavior_tree = pose, behavior_tree
        return self._send(self.nav_to_pose_client, goal)

    def spin(self, spin_dist=1.57, time_allowance=10):
        goal = Spin.Goal()
        goal.target_yaw = float(spin_dist)
        goal.time_allowance.sec = time_allowance
        return self._send(self.spin_client, goal)


class Mission:
    def __init__(self, navigator, sensors):
        self.navigator, self.sensors = navigator, sensors
        self.clock = MissionClock(sensors.param('mission_duration_sec'))
        for key in ('return_speed_mps', 'goal_timeout_sec', 'return_timeout_sec',
                    'failed_cooldown_sec', 'save_interval_sec', 'footprint_radius_m',
                    'sensor_timeout_sec', 'observe_sec', 'return_reserve_sec',
                    'turn_speed_rps', 'planning_allowance_sec', 'recovery_allowance_sec',
                    'late_return_timeout_sec', 'selection_timeout_sec'):
            if not math.isfinite(sensors.param(key)) or sensors.param(key) <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.home_future = None
        self.home_plan_stamp = 0.0
        self.home_plan_distance = 0.0
        self.odom_distance = 0.0
        self.last_odom = None
        self.budget = {}
        self.arrival = None
        self.export_completed = None
        self.cleanup_errors = []
        self.return_latched = False
        self.return_started = None
        self.selection_deadline = None
        self.home_request_stamp = 0.0
        self.home_request_distance = 0.0
        self.motion_samples = []
        if sensors.param('speed_profiles_enabled'):
            if not (0 < sensors.param('exploration_speed_mps') <= sensors.param('return_limit_mps') <= 0.22):
                raise ValueError('Measured speed caps must satisfy 0 < exploration <= return <= 0.22')
        self.state = 'READY'
        self.reason = ''
        self.home = None
        self.home_seconds = math.inf
        self.cooldown = []
        self.last_save = time.monotonic()
        self.last_home_check = 0.0
        self.last_status = 0.0
        self.returned = False
        self.map_saved = False
        self.tags_saved = False
        self.active = False
        self.checkpoints = {}
        self.output = Path(sensors.param('output_dir')).expanduser()

    def pose(self, x=None, y=None, yaw=0.0):
        p = PoseStamped()
        p.header.frame_id = self.sensors.param('map_frame')
        p.header.stamp = self.navigator.get_clock().now().to_msg()
        if x is None:
            tf = self.sensors.buffer.lookup_transform(
                p.header.frame_id, self.sensors.param('base_frame'), Time())
            age = (self.navigator.get_clock().now().nanoseconds -
                   Time.from_msg(tf.header.stamp).nanoseconds) / 1e9
            if abs(age) > self.sensors.param('sensor_timeout_sec'):
                raise RuntimeError('Robot transform is stale')
            p.pose.position.x = tf.transform.translation.x
            p.pose.position.y = tf.transform.translation.y
            p.pose.orientation = tf.transform.rotation
        else:
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return p

    def healthy(self):
        grid, _, _, receipts, camera, _ = self.sensors.snapshot()
        now = time.monotonic()
        for name in ('scan', 'odom', 'camera', 'detections'):
            if now-receipts.get(name, 0) > self.sensors.param('sensor_timeout_sec'):
                return False
        if (grid is None or now-receipts.get('map', 0) > 20.0 or
                grid.info.resolution <= 0 or grid.header.frame_id != self.sensors.param('map_frame')):
            return False
        if camera is None or camera.k[0] <= 0 or camera.k[4] <= 0:
            return False
        try:
            self.pose()
            return self.sensors.buffer.can_transform(
                self.sensors.param('base_frame'), camera.header.frame_id, Time())
        except (TransformException, RuntimeError):
            return False

    def wait(self, future, timeout=3.0):
        deadline = time.monotonic()+timeout
        if self.selection_deadline is not None:
            deadline = min(deadline, self.selection_deadline)
        if self.state == 'RETURN' and self.return_started is not None:
            deadline = min(deadline, self.return_deadline())
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if self.state == 'RETURN' and self.return_started is not None and self.return_expired():
                break
            if self.active and not self.healthy():
                self.cancel()
                raise RuntimeError('Sensor/TF stream lost during planning')
            if self.active and self.state != 'RETURN' and self.sensors.return_requested.is_set():
                self.cancel()
            self.publish()
            rclpy.spin_once(self.navigator, timeout_sec=0.02)
            # The sensor executor resolves this future independently.
        return future.result() if future.done() else None

    def plan(self, start, goal):
        if self.home_future is not None and not self.home_future.done():
            self.wait(self.home_future, 3.2)
            if not self.home_future.done():
                return None
        if self.selection_deadline is not None and time.monotonic() >= self.selection_deadline:
            return None
        client = self.sensors.planner
        if not client.server_is_ready():
            return None
        request = ComputePathToPose.Goal()
        request.start, request.goal, request.use_start = start, goal, True
        future = client.send_goal_async(request)
        handle = self.wait(future)
        if handle is None:
            # Cancel even if the server accepts after our deadline.
            def cancel_late(done):
                result = done.result()
                if result and result.accepted:
                    result.cancel_goal_async()
            future.add_done_callback(cancel_late)
            return None
        if not handle.accepted:
            return None
        result = self.wait(handle.get_result_async())
        if result is None:
            handle.cancel_goal_async()
            return None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        grid = self.sensors.snapshot()[0]
        path = result.result.path
        return path if grid and path_is_known(
            grid, path, self.sensors.param('footprint_radius_m')) else None

    def publish(self):
        now = time.monotonic()
        if now-self.last_status < 1.0:
            return
        self.last_status = now
        self.speed_profile()
        if self.clock.started is not None:
            self.update_distance()
        self.checkpoint(now)
        self.sensors.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'ready': self.sensors.ready, 'reason': self.reason,
            'elapsed_sec': self.clock.elapsed(now), 'remaining_sec': self.clock.remaining(now),
            **self.result_details(),
            'estimated_home_sec': self.home_seconds if math.isfinite(self.home_seconds) else None,
            **self.sensors.snapshot()[5]})))

    def route_budget(self, path, start):
        q = start.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return return_budget(path, self.sensors.param('return_speed_mps'),
                             self.sensors.param('turn_speed_rps'), yaw,
                             self.sensors.param('planning_allowance_sec'),
                             self.sensors.param('recovery_allowance_sec'),
                             self.sensors.param('return_reserve_sec'))

    def background_home_plan(self, start):
        # Only the sensor executor services these futures; never spin navigator here.
        client = self.sensors.planner
        request = ComputePathToPose.Goal()
        request.start, request.goal, request.use_start = start, self.home, True
        future = client.send_goal_async(request)
        deadline = time.monotonic()+3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.add_done_callback(lambda f: f.result().cancel_goal_async()
                                     if f.result() and f.result().accepted else None)
            return None
        handle = future.result()
        if not handle or not handle.accepted:
            return None
        result = handle.get_result_async()
        while not result.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not result.done():
            handle.cancel_goal_async()
            return None
        response = result.result()
        grid = self.sensors.snapshot()[0]
        path = response.result.path
        if response.status != GoalStatus.STATUS_SUCCEEDED or grid is None or not path_is_known(
                grid, path, self.sensors.param('footprint_radius_m')):
            return None
        return self.route_budget(path, start)

    def update_distance(self):
        odom = self.sensors.odom
        if odom is not None:
            p = odom.pose.pose.position
            current = (p.x, p.y)
            if self.last_odom is not None:
                self.odom_distance += math.dist(current, self.last_odom)
            self.last_odom = current

    def check_return(self, force=False):
        now = time.monotonic()
        if self.return_latched:
            return True
        if self.sensors.return_requested.is_set():
            self.reason = 'Operator requested return'
            self.return_latched = True
            return True
        self.update_distance()
        if self.home_future is not None and self.home_future.done():
            try:
                result = self.home_future.result()
            except Exception:
                result = None
            self.home_future = None
            if result is None:
                self.reason = 'No verified home path; stop exploration'
                self.return_latched = True
                return True
            self.budget = result
            self.home_plan_stamp = self.home_request_stamp
            self.home_plan_distance = self.home_request_distance
        if self.home_plan_stamp == 0.0:
            start = self.pose()
            path = self.plan(start, self.home)
            if path is None:
                self.reason = 'No verified home path; stop exploration'
                self.return_latched = True
                return True
            self.budget = self.route_budget(path, start)
            self.home_plan_stamp = now
            self.home_plan_distance = self.odom_distance
        if now-self.home_plan_stamp > 10.0:
            self.reason = 'Home plan stale'
            self.return_latched = True
            return True
        if self.home_future is None and now-self.home_plan_stamp >= 5.0:
            self.last_home_check = now
            self.home_request_distance = self.odom_distance
            self.home_request_stamp = now
            self.home_future = self.worker.submit(self.background_home_plan, self.pose())
        extra = max(0.0, self.odom_distance-self.home_plan_distance)/self.sensors.param('return_speed_mps')
        self.home_seconds = self.budget['total_sec'] + extra - self.sensors.param('return_reserve_sec')
        if not self.clock.fits(now, self.home_seconds+self.sensors.param('return_reserve_sec')):
            self.reason = 'Return reserve reached'
            self.return_latched = True
            return True
        return False

    def speed_profile(self):
        if not self.sensors.param('speed_profiles_enabled'):
            return
        from nav2_msgs.msg import SpeedLimit
        msg = SpeedLimit()
        msg.header.stamp = self.navigator.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = self.sensors.param(
            'return_limit_mps' if self.state == 'RETURN' else 'exploration_speed_mps')
        self.sensors.speed_pub.publish(msg)

    def return_deadline(self):
        limit = (self.clock.started+self.clock.duration if self.clock.duration else self.return_started)
        return limit+self.sensors.param('late_return_timeout_sec')

    def return_expired(self):
        return time.monotonic() >= self.return_deadline()

    def verify_home(self):
        stable = None
        deadline = time.monotonic()+2.0
        if self.active:
            return False
        while rclpy.ok() and time.monotonic() < deadline:
            if self.return_started is not None and self.return_expired():
                return False
            if not self.healthy():
                return False
            p = self.pose().pose.position
            odom = self.sensors.odom
            near = math.hypot(p.x-self.home.pose.position.x, p.y-self.home.pose.position.y) <= 0.25
            stopped = odom is not None and math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y) < 0.02 and abs(odom.twist.twist.angular.z) < 0.05
            if not near or not stopped:
                stable = None
            elif stable is None:
                stable = time.monotonic()
            elif time.monotonic()-stable >= 0.5:
                self.arrival = self.clock.elapsed(time.monotonic())
                self.returned = True
                return True
            time.sleep(0.05)
        return False

    def cancel(self):
        if not self.active:
            return
        # Bounded cancellation instead of BasicNavigator.cancelTask's unbounded wait.
        self.navigator.goal_handle.cancel_goal_async()
        deadline = time.monotonic()+5.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self.navigator.isTaskComplete():
                self.active = False
                return
        raise RuntimeError('Navigation did not acknowledge cancellation; refusing another goal')

    def navigate(self, goal=None, observe=False, returning=False):
        if not self.healthy():
            raise RuntimeError('Sensor/TF readiness lost before motion')
        client = self.navigator.spin_client if observe else self.navigator.nav_to_pose_client
        if not client.server_is_ready():
            raise RuntimeError('Navigation action server is unavailable')
        if goal:
            goal = copy.deepcopy(goal)
            goal.header.stamp = self.navigator.get_clock().now().to_msg()
        if returning and self.return_expired():
            return False
        self.navigator.acceptance_timeout = (min(3.0, max(0.0, self.return_deadline()-time.monotonic()))
                                             if returning else 3.0)
        self.speed_profile()
        accepted = (self.navigator.spin(spin_dist=2*math.pi,
                                        time_allowance=int(self.sensors.param('observe_sec')))
                    if observe else self.navigator.goToPose(goal))
        if accepted is False:
            return False
        self.active = True
        started = time.monotonic()
        timeout = self.sensors.param('observe_sec' if observe else
                                     'return_timeout_sec' if returning else 'goal_timeout_sec')
        while rclpy.ok() and not self.navigator.isTaskComplete():
            self.publish()
            if not self.healthy():
                self.cancel()
                raise RuntimeError('Sensor or transform stream became stale')
            if not returning and self.check_return():
                self.cancel()
                return False
            if time.monotonic()-started > timeout or (
                    returning and self.return_expired()):
                self.cancel()
                return False
            # Save between actions; disk/service delays must not block motion supervision.
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped during motion')
        self.active = False
        self.motion_samples.append(dict(returning=returning, observation=observe,
                                        duration_sec=time.monotonic()-started))
        return self.navigator.getResult() == TaskResult.SUCCEEDED

    def select_goal(self):
        grid, markers, revision, _, _, _ = self.sensors.snapshot()
        if grid is None or markers is None or markers.header.frame_id != self.sensors.param('map_frame'):
            return None, revision, False
        now = time.monotonic()
        self.cooldown = [(x, y, until) for x, y, until in self.cooldown if until > now]
        start = self.pose()
        candidates = []
        waiting = False
        for point in markers.points:
            if any(math.hypot(point.x-x, point.y-y) < 0.5 for x, y, _ in self.cooldown):
                waiting = True
                continue
            # Stand back from the unknown boundary, then face it.
            choices = []
            for distance in (0.2, 0.35, 0.5):
                for i in range(8):
                    angle = i*math.pi/4
                    x, y = point.x+distance*math.cos(angle), point.y+distance*math.sin(angle)
                    if clear_point(grid, x, y, self.sensors.param('footprint_radius_m')):
                        choices.append((math.hypot(x-start.pose.position.x, y-start.pose.position.y), x, y))
            for _, x, y in sorted(choices)[:3]:
                candidates.append((math.hypot(x-start.pose.position.x, y-start.pose.position.y),
                                   self.pose(x, y, math.atan2(point.y-y, point.x-x)), point))
        best = None
        self.selection_deadline = time.monotonic()+self.sensors.param('selection_timeout_sec')
        # Each planning request has a timeout; check operator/deadline between candidates.
        for _, goal, point in sorted(candidates, key=lambda c: c[0])[:6]:
            if time.monotonic() >= self.selection_deadline:
                break
            if self.sensors.return_requested.is_set():
                break
            if not self.clock.fits(time.monotonic(), self.home_seconds+self.sensors.param('return_reserve_sec')):
                break
            path = self.plan(start, goal)
            if path is None:
                continue
            length = path_length(path)
            back = self.plan(goal, self.home)
            if back is None:
                continue
            total = ((length+path_length(back))/self.sensors.param('return_speed_mps') +
                     self.sensors.param('observe_sec') + self.sensors.param('return_reserve_sec'))
            if self.clock.fits(time.monotonic(), total) and (best is None or length < best[0]):
                best = (length, goal, point)
        self.selection_deadline = None
        return best, revision, waiting

    def checkpoint(self, now):
        if self.clock.started is None or self.state in ('READY', 'SAVE', 'FINISHED', 'FAILED'):
            return
        for name, (future, started) in list(self.checkpoints.items()):
            if future.done():
                result = future.result()
                success = bool(result and (result.result if name == 'map' else result.success))
                if not success:
                    self.navigator.get_logger().error(f'Periodic {name} export failed')
                del self.checkpoints[name]
            elif now-started > 10.0:
                # Keep the in-flight request to avoid flooding an unresponsive service.
                self.navigator.get_logger().error(f'Periodic {name} export is not responding')
                self.checkpoints[name] = (future, now)
        if now-self.last_save < self.sensors.param('save_interval_sec'):
            return
        self.last_save = now
        if 'tags' not in self.checkpoints and self.sensors.save_tags.service_is_ready():
            self.checkpoints['tags'] = (self.sensors.save_tags.call_async(Trigger.Request()), now)
        if 'map' not in self.checkpoints and self.sensors.save_map.service_is_ready():
            self.checkpoints['map'] = (self.sensors.save_map.call_async(self.map_request()), now)

    def map_request(self):
        req = SaveMap.Request()
        req.map_topic = '/map'
        req.map_url = str(self.output/'map')
        req.image_format, req.map_mode = 'pgm', 'trinary'
        req.free_thresh, req.occupied_thresh = 0.25, 0.65
        return req

    def export_wait(self, client, request, timeout):
        try:
            if not client.service_is_ready():
                self.cleanup_errors.append('Export service unavailable')
                return None
            future = client.call_async(request)
            deadline = time.monotonic()+timeout
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self.navigator, timeout_sec=0.02)
            result = future.result() if future.done() else None
            if result is None:
                self.cleanup_errors.append('Export service timed out')
            return result
        except Exception as error:
            self.cleanup_errors.append(str(error))
            return None

    def save(self):
        self.last_save = time.monotonic()
        for future, _ in list(self.checkpoints.values()):
            try:
                self.wait(future, 7.0)
            except Exception as error:
                self.cleanup_errors.append(str(error))
        self.checkpoints.clear()
        if self.sensors.save_tags.service_is_ready():
            result = self.export_wait(self.sensors.save_tags, Trigger.Request(), 5.0)
            self.tags_saved = bool(result and result.success)
        else:
            self.tags_saved = False
        if self.sensors.save_map.service_is_ready():
            req = self.map_request()
            result = self.export_wait(self.sensors.save_map, req, 7.0)
            self.map_saved = bool(result and result.result)
        else:
            self.map_saved = False
        self.export_completed = self.clock.elapsed(time.monotonic())
        atomic_yaml(self.output/'mission_result.yaml', {
            'state': self.state, 'reason': self.reason, 'returned_home': self.returned,
            **self.result_details(),
            'elapsed_sec': self.clock.elapsed(time.monotonic()),
            'duration_sec': self.clock.duration, 'occupancy_saved': self.map_saved,
            'semantic_saved': self.tags_saved, 'tags': self.sensors.snapshot()[5]})

    def run(self):
        # Unlike the original unbounded activation wait, READY remains inspectable.
        while rclpy.ok() and not self.sensors.start_requested.is_set():
            self.sensors.ready = ((not self.sensors.param('challenge_mode') or self.clock.duration > 0)
                                  and self.healthy() and self.navigator.nav_to_pose_client.server_is_ready()
                                  and self.navigator.spin_client.server_is_ready()
                                  and self.sensors.planner.server_is_ready()
                                  and self.sensors.nodes_active()
                                  and self.sensors.start_tags.service_is_ready()
                                  and self.sensors.save_map.service_is_ready())
            self.publish()
            time.sleep(0.1)
        if not rclpy.ok():
            return
        if self.sensors.param('challenge_mode') and self.clock.duration <= 0:
            raise RuntimeError('Challenge mode requires a positive mission duration')
        self.clock.started = self.sensors.start_time
        self.sensors.ready = False
        self.home = self.pose()
        self.output.mkdir(parents=True, exist_ok=True)
        atomic_yaml(self.output/'home.yaml', {
            'frame_id': self.home.header.frame_id,
            'position': {'x': self.home.pose.position.x, 'y': self.home.pose.position.y},
            'orientation': {k: getattr(self.home.pose.orientation, k) for k in ('x', 'y', 'z', 'w')}})
        result = self.wait(self.sensors.start_tags.call_async(Trigger.Request()))
        if not result or not result.success:
            raise RuntimeError('Could not start tag recording')
        empty_count, last_revision = 0, -1
        while rclpy.ok():
            self.state = 'EXPLORE'
            if not self.healthy():
                raise RuntimeError('Sensor/TF readiness lost')
            if self.check_return(force=True):
                break
            selected, revision, waiting = self.select_goal()
            if selected is None:
                if revision != last_revision:
                    empty_count = 0 if waiting else empty_count+1
                    last_revision = revision
                if empty_count >= 3:
                    self.reason = 'No usable frontier in three map updates'
                    break
                self.publish()
                time.sleep(0.2)
            else:
                empty_count = 0
                _, goal, frontier = selected
                if self.check_return():
                    break
                start = self.pose()
                self.selection_deadline = time.monotonic()+self.sensors.param('selection_timeout_sec')
                outward = self.plan(start, goal)
                back = self.plan(goal, self.home) if outward is not None else None
                self.selection_deadline = None
                if back is None or not self.clock.fits(time.monotonic(),
                        self.route_budget(outward, start)['total_sec'] +
                        self.route_budget(back, goal)['travel_sec'] +
                        self.route_budget(back, goal)['turning_sec'] + self.sensors.param('observe_sec')):
                    self.reason = 'Selected excursion no longer fits'
                    self.return_latched = True
                    break
                success = self.navigate(goal)
                self.cooldown.append((frontier.x, frontier.y,
                                      time.monotonic()+self.sensors.param('failed_cooldown_sec')))
                if self.check_return():
                    break
                if success and self.clock.fits(time.monotonic(), self.home_seconds+
                                                self.sensors.param('return_reserve_sec')+
                                                self.sensors.param('observe_sec')):
                    self.state = 'OBSERVE'
                    self.navigate(observe=True)
            self.publish()
        self.state = 'RETURN'
        self.return_latched = True
        self.return_started = time.monotonic()
        self.cancel()
        self.speed_profile()
        for _ in range(3):
            if not rclpy.ok() or self.return_expired():
                break
            if self.verify_home():
                break
            if self.plan(self.pose(), self.home):
                self.navigate(self.home, returning=True)
                if self.verify_home():
                    break
            time.sleep(1.0)
        self.state = 'SAVE'
        self.speed_profile()
        self.export_wait(self.sensors.stop_tags, Trigger.Request(), 5.0)
        self.save()
        self.state = 'FINISHED' if self.returned and (not self.clock.duration or self.arrival <= self.clock.duration) and self.map_saved and self.tags_saved else 'FAILED'
        self.save_result()
        self.last_status = 0.0
        self.publish()
        self.navigator.get_logger().info(f'Mission {self.state}; results: {self.output}')

    def failure_cleanup(self):
        self.selection_deadline = None
        for operation in (self.cancel,
                          lambda: self.export_wait(self.sensors.stop_tags, Trigger.Request(), 5.0),
                          self.save):
            try:
                operation()
            except Exception as error:
                self.cleanup_errors.append(str(error))
                self.navigator.get_logger().error(f'Cleanup failed: {error}')
        if not self.active:
            self.speed_profile()
        self.save_result()

    def result_details(self):
        return dict(return_budget=self.budget,
                    home_plan_age_sec=(time.monotonic()-self.home_plan_stamp if self.home_plan_stamp else None),
                    requested_speed_profile=('return' if self.state == 'RETURN' else 'exploration')
                        if self.sensors.param('speed_profiles_enabled') else 'disabled',
                    home_arrival_elapsed_sec=self.arrival,
                    returned_before_deadline=(self.arrival is not None and self.arrival <= self.clock.duration)
                        if self.clock.duration else None,
                    export_completion_elapsed_sec=self.export_completed,
                    cleanup_errors=list(self.cleanup_errors), odometry_distance_m=self.odom_distance,
                    motion_samples=self.motion_samples)

    def save_result(self):
        atomic_yaml(self.output/'mission_result.yaml', {
            'state': self.state, 'reason': self.reason, 'returned_home': self.returned,
            **self.result_details(),
            'elapsed_sec': self.clock.elapsed(time.monotonic()),
            'duration_sec': self.clock.duration, 'occupancy_saved': self.map_saved,
            'semantic_saved': self.tags_saved, 'tags': self.sensors.snapshot()[5]})


def main():
    rclpy.init()
    navigator = BoundedNavigator()
    sensors = MissionSensors().start()
    mission = None
    exit_code = 0
    try:
        mission = Mission(navigator, sensors)
        mission.run()
        exit_code = 1 if mission.state == 'FAILED' else 0
    except (KeyboardInterrupt, Exception) as error:
        exit_code = 1
        navigator.get_logger().error(f'Mission interrupted: {error}')
        if mission:
            mission.state, mission.reason = 'FAILED', str(error) or 'Operator interruption'
            try:
                if mission.clock.started is not None and rclpy.ok():
                    mission.failure_cleanup()
            except Exception as error:
                navigator.get_logger().error(f'Result export failed: {error}')
    finally:
        if mission:
            mission.worker.shutdown(wait=True, cancel_futures=True)
        sensors.stop()
        navigator.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
