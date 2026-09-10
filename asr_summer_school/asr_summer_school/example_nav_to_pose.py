#!/usr/bin/env python3
"""Single-file onboard mission; no build or other repository modules required.

Start robot drivers, SLAM, Nav2 and apriltag_ros first. On nuc01:
  source /opt/ros/humble/setup.bash
  source ~/ros_ws/install/setup.bash
  python3 /path/to/example_nav_to_pose.py --duration 240
Use the existing ROS_DOMAIN_ID=1 / rmw_zenoh_cpp environment on both machines.
Press Enter when ready to record home and start the clock. Ctrl+C cancels motion
and saves available results; it does not shut down the shared robot stack.
Laptop: ros2 topic echo /mission/status; RViz MarkerArray /mission/tag_markers.
Copy the printed output directory with scp after completion. No map saver needed.
Camera calibration and Nav2 footprint/speed limits belong to the running stack.
"""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
from pathlib import Path
import statistics
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from sensor_msgs.msg import LaserScan, CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def cell(grid, x, y):
    angle = yaw(grid.info.origin.orientation)
    dx, dy = x-grid.info.origin.position.x, y-grid.info.origin.position.y
    r = grid.info.resolution
    return (math.floor((math.cos(angle)*dx+math.sin(angle)*dy)/r),
            math.floor((-math.sin(angle)*dx+math.cos(angle)*dy)/r))


def world(grid, x, y):
    angle = yaw(grid.info.origin.orientation)
    dx, dy = (x+.5)*grid.info.resolution, (y+.5)*grid.info.resolution
    return (grid.info.origin.position.x+math.cos(angle)*dx-math.sin(angle)*dy,
            grid.info.origin.position.y+math.sin(angle)*dx+math.cos(angle)*dy)


def free(grid, x, y):
    return (0 <= x < grid.info.width and 0 <= y < grid.info.height and
            0 <= grid.data[y*grid.info.width+x] <= 25)


def clear(grid, x, y, radius):
    cx, cy = cell(grid, x, y)
    n = math.ceil(radius/grid.info.resolution)
    # Conservative square footprint; unknown and outside-map cells are blocked.
    return all(free(grid, cx+dx, cy+dy)
               for dx in range(-n, n+1) for dy in range(-n, n+1))


def known_path(grid, path, radius):
    if not path or not path.poses:
        return False
    previous = path.poses[0].pose.position
    for pose in path.poses:
        p = pose.pose.position
        steps = max(1, math.ceil(math.hypot(p.x-previous.x, p.y-previous.y)/
                                 (grid.info.resolution*.5)))
        for i in range(steps+1):
            if not clear(grid, previous.x+(p.x-previous.x)*i/steps,
                         previous.y+(p.y-previous.y)*i/steps, radius):
                return False
        previous = p
    return True


def route_seconds(path, speed=.10, turn_speed=.4):
    points = [p.pose.position for p in path.poses]
    distance = sum(math.hypot(b.x-a.x, b.y-a.y) for a, b in zip(points, points[1:]))
    heading = yaw(path.poses[0].pose.orientation) if points else 0.
    turns = 0.
    if points:
        anchor = points[0]
        for p in points[1:]:
            if math.hypot(p.x-anchor.x, p.y-anchor.y) >= .25:
                next_heading = math.atan2(p.y-anchor.y, p.x-anchor.x)
                turns += abs(math.atan2(math.sin(next_heading-heading), math.cos(next_heading-heading)))
                heading, anchor = next_heading, p
    return distance/speed+turns/turn_speed


def frontiers(grid, position, radius, deadline):
    """Flood reachable free space; select clear viewpoints near unknown cells."""
    start = cell(grid, position.x, position.y)
    queue, seen, candidates = deque([start]), {start}, []
    buckets = set()
    while queue and time.monotonic() < deadline:
        x, y = queue.popleft()
        if not free(grid, x, y):
            continue
        neighbors = [(x+1,y), (x-1,y), (x,y+1), (x,y-1)]
        boundary = any(0 <= a < grid.info.width and 0 <= b < grid.info.height and
                       grid.data[b*grid.info.width+a] == -1 for a,b in neighbors)
        if boundary:
            wx, wy = world(grid, x, y)
            bucket = (math.floor(wx/.5), math.floor(wy/.5))
            if bucket not in buckets:
                buckets.add(bucket)
                for distance in (.3, .5, .7):
                    for i in range(8):
                        angle = i*math.pi/4
                        px, py = wx+distance*math.cos(angle), wy+distance*math.sin(angle)
                        if clear(grid, px, py, radius):
                            candidates.append((math.hypot(px-position.x, py-position.y), px, py,
                                               math.atan2(wy-py, wx-px)))
        for point in neighbors:
            if point not in seen and free(grid, *point):
                seen.add(point)
                queue.append(point)
    return sorted(candidates)


def summarize(records):
    tags = []
    for (family, tag_id), samples in sorted(records.items()):
        center = [statistics.median(p[i] for p in samples) for i in range(3)]
        inliers = [p for p in samples if math.dist(p, center) <= .30]
        tags.append(dict(family=family, id=tag_id, position=dict(zip(('x','y','z'),center)),
                         observations=len(samples), confirmed=len(inliers)>=3))
    return tags


def atomic(path, data):
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_bytes(data)
    temporary.replace(path)


def export(output, grid, tags, result):
    """JSON is valid YAML; only the occupancy image needs binary serialization."""
    output.mkdir(parents=True, exist_ok=True)
    errors = []
    result = dict(result, occupancy_saved=False, semantic_saved=False)
    try:
        if grid is not None:
            pixels = bytearray()
            for y in reversed(range(grid.info.height)):
                for x in range(grid.info.width):
                    value = grid.data[y*grid.info.width+x]
                    pixels.append(254 if 0 <= value <= 25 else 0 if value >= 65 else 205)
            # Unique image name keeps the previous YAML/image pair valid during saving.
            image_name = f'map_{time.time_ns()}.pgm'
            atomic(output/image_name, f'P5\n{grid.info.width} {grid.info.height}\n255\n'.encode()+pixels)
            metadata = dict(image=image_name, resolution=grid.info.resolution,
                            origin=[grid.info.origin.position.x, grid.info.origin.position.y,
                                    yaw(grid.info.origin.orientation)],
                            negate=0, occupied_thresh=.65, free_thresh=.25, mode='trinary')
            atomic(output/'map.yaml', json.dumps(metadata, indent=2).encode())
        result['occupancy_saved'] = grid is not None
        if grid is None:
            errors.append('No occupancy map available')
    except OSError as error:
        errors.append(f'Occupancy export: {error}')
    try:
        atomic(output/'semantic_map.yaml', json.dumps(tags, indent=2).encode())
        result['semantic_saved'] = True
    except OSError as error:
        errors.append(f'Semantic export: {error}')
    if errors:
        result['state'] = 'FAILED'
    result['export_errors'] = errors
    atomic(output/'mission_result.yaml', json.dumps(result, indent=2).encode())
    if errors:
        raise OSError('; '.join(errors))


class Sensors(Node):
    def __init__(self, args):
        super().__init__('python_mission')
        self.args = args
        self.lock = threading.RLock()
        self.grid = self.odom = self.camera = None
        self.receipts = {}
        self.records, self.seen = {}, {}
        self.pending = deque(maxlen=1000)
        self.raw = deque(maxlen=10000)
        self.dropped = 0
        self.recording = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for kind, topic, name, qos in (
                (OccupancyGrid, args.map_topic, 'grid', latched),
                (Odometry, args.odom_topic, 'odom', qos_profile_sensor_data),
                (LaserScan, args.scan_topic, 'scan', qos_profile_sensor_data),
                (CameraInfo, args.camera_info_topic, 'camera', qos_profile_sensor_data)):
            self.create_subscription(kind, topic, lambda msg, n=name: self.cache(n,msg), qos)
        self.create_subscription(AprilTagDetectionArray, args.detections_topic,
                                 self.detect, qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/mission/status', 10)
        self.markers = self.create_publisher(MarkerArray, '/mission/tag_markers', latched)
        self.navigate = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        self.create_timer(.05, self.process_tags)
        self._mission_executor = SingleThreadedExecutor()
        self._mission_executor.add_node(self)
        self.thread = threading.Thread(target=self._mission_executor.spin, daemon=True)
        self.thread.start()

    def cache(self, name, msg):
        with self.lock:
            setattr(self, name, msg)
            self.receipts[name] = time.monotonic()

    def pose(self):
        tf = self.buffer.lookup_transform(self.args.map_frame, self.args.base_frame, Time())
        age = abs(self.get_clock().now().nanoseconds-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
        if age > 3.:
            raise RuntimeError('stale map-to-base TF')
        pose = PoseStamped()
        pose.header.frame_id = self.args.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.orientation = tf.transform.rotation
        return pose

    def issues(self, perception=True):
        now = time.monotonic()
        with self.lock:
            issues = [f'{name}: missing/stale' for name in
                      (('scan','odom','camera','detections') if perception else ('scan','odom'))
                      if now-self.receipts.get(name,0) > 3.]
            grid = self.grid
            if (grid is None or grid.info.resolution <= 0 or
                    grid.header.frame_id != self.args.map_frame or
                    len(grid.data) != grid.info.width*grid.info.height or
                    now-self.receipts.get('grid',0) > 20.):
                issues.append('map: missing/stale/invalid')
            if perception and (self.camera is None or self.camera.k[0] <= 0 or self.camera.k[4] <= 0):
                issues.append('camera intrinsics unavailable')
        try:
            self.pose()
            if perception and self.camera is not None and not self.buffer.can_transform(
                    self.args.base_frame, self.camera.header.frame_id, Time()):
                issues.append('camera-to-base TF unavailable')
        except (TransformException, RuntimeError) as error:
            issues.append(str(error))
        for name, client in (('navigate_to_pose',self.navigate), ('compute_path_to_pose',self.planner)):
            if not client.server_is_ready():
                issues.append(f'action unavailable: {name}')
        return issues

    def detect(self, msg):
        with self.lock:
            self.receipts['detections'] = time.monotonic()
            if not self.recording:
                return
            stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if not stamp:
                return
            for tag in msg.detections:
                key = (tag.family, int(tag.id))
                if tag.hamming or self.seen.get(key) == stamp:
                    continue
                self.seen[key] = stamp
                if len(self.pending) == self.pending.maxlen:
                    self.dropped += 1
                self.pending.append((time.monotonic(), stamp, key))

    def process_tags(self):
        with self.lock:
            for _ in range(len(self.pending)):
                receipt, stamp, key = self.pending.popleft()
                try:
                    tf = self.buffer.lookup_transform(self.args.map_frame, f'{key[0]}:{key[1]}',
                                                      Time(nanoseconds=stamp))
                except TransformException:
                    if time.monotonic()-receipt < 1.:
                        self.pending.append((receipt,stamp,key))
                    else:
                        self.dropped += 1
                    continue
                p = tf.transform.translation
                point = [p.x,p.y,p.z]
                if not all(math.isfinite(v) for v in point):
                    self.dropped += 1
                    continue
                self.records.setdefault(key,deque(maxlen=300)).append(point)
                if len(self.raw) == self.raw.maxlen:
                    self.dropped += 1
                self.raw.append(dict(family=key[0],id=key[1],stamp_ns=stamp,position=point))

    def stop(self):
        self._mission_executor.shutdown(timeout_sec=3.)
        self.thread.join(timeout=3.)
        self.destroy_node()


class Mission:
    def __init__(self, args):
        self.args = args
        self.s = Sensors(args)
        self.started = None
        self.home = None
        self.state, self.reason = 'READY', ''
        self.active = None
        self.home_budget = 60.
        self.home_checked = 0.
        self.arrival = None
        self.last_status = self.last_save = 0.
        self.cooldown = []
        self.output = Path(args.output).expanduser()/time.strftime('%Y%m%d_%H%M%S')
        self.output = self.output.with_name(self.output.name+f'_{time.time_ns()%10**9:09d}')
        self.writer = ThreadPoolExecutor(max_workers=1)
        self.saving = None
        self.export_error = None
        self.pending_actions = []

    def remaining(self):
        return self.args.duration if self.started is None else self.args.duration-(time.monotonic()-self.started)

    def publish(self, issues=None):
        if time.monotonic()-self.last_status < 1.:
            return
        self.last_status = time.monotonic()
        with self.s.lock:
            tags = summarize(self.s.records)
        status = dict(state=self.state, reason=self.reason, readiness_issues=issues or [],
                      ready=self.state == 'READY' and issues is not None and not issues,
                      remaining_sec=max(0.,self.remaining()), expected_tag_count=self.args.expected_tags,
                      confirmed=sum(t['confirmed'] for t in tags), unique=len(tags),
                      estimated_home_sec=self.home_budget, output_dir=str(self.output),
                      returned_home=self.arrival is not None,
                      returned_before_deadline=self.arrival is not None and self.arrival<=self.args.duration,
                      export_error=self.export_error)
        self.s.status.publish(String(data=json.dumps(status)))
        print(json.dumps(status), flush=True)
        markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)
        for tag in tags:
            marker = Marker()
            marker.header.frame_id = self.args.map_frame
            marker.header.stamp = self.s.get_clock().now().to_msg()
            marker.ns, marker.id = tag['family'], tag['id']
            marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            marker.pose.orientation.w = 1.
            for axis,value in tag['position'].items():
                setattr(marker.pose.position,axis,value)
            marker.scale.z = .18
            marker.color.r = .1 if tag['confirmed'] else 1.
            marker.color.g, marker.color.a = .9, 1.
            marker.text = f"{tag['family']}:{tag['id']}"+('' if tag['confirmed'] else ' provisional')
            markers.markers.append(marker)
        self.s.markers.publish(markers)

    def wait(self, future, seconds, supervise=True):
        end = time.monotonic()+seconds
        while rclpy.ok() and not future.done() and time.monotonic()<end:
            if supervise and self.started is not None:
                if self.s.issues(perception=False):
                    raise RuntimeError('Navigation sensors/TF lost')
                if self.state != 'RETURN' and self.remaining() <= self.home_budget+5.:
                    break
            self.publish()
            time.sleep(.02)
        return future.result() if future.done() else None

    def submit(self, client, goal):
        future = client.send_goal_async(goal)
        self.pending_actions.append(future)
        try:
            handle = self.wait(future, 3.)
            if handle is None:
                raise RuntimeError('Action acceptance timed out; late acceptance will be canceled')
        except BaseException:
            def late(done):
                accepted = done.result()
                if accepted and accepted.accepted:
                    accepted.cancel_goal_async()
            future.add_done_callback(late)
            raise
        self.pending_actions.remove(future)
        return handle if handle.accepted else None

    def drain_pending(self):
        for future in self.pending_actions:
            handle = self.wait(future, 5., supervise=False)
            if handle is None:
                raise RuntimeError('Action acceptance still unresolved during shutdown')
            if handle.accepted:
                self.wait(handle.cancel_goal_async(), 3., supervise=False)
                if self.wait(handle.get_result_async(), 3., supervise=False) is None:
                    raise RuntimeError('Late action cancellation not confirmed')
        self.pending_actions.clear()

    def cancel(self):
        if self.active is None:
            return
        handle = self.active
        self.wait(handle.cancel_goal_async(), 3., supervise=False)
        result = self.wait(handle.get_result_async(), 3., supervise=False)
        if result is None:
            raise RuntimeError('Motion cancellation not confirmed; refusing another goal')
        self.active = None

    def plan(self, start, goal):
        request = ComputePathToPose.Goal()
        request.start, request.goal, request.use_start = start, goal, True
        handle = self.submit(self.s.planner, request)
        if handle is None:
            return None
        result = self.wait(handle.get_result_async(), 3.)
        if result is None:
            handle.cancel_goal_async()
            raise RuntimeError('Planner timed out')
        with self.s.lock:
            grid = self.s.grid
        if result.status != GoalStatus.STATUS_SUCCEEDED or grid is None:
            return None
        return result.result.path if known_path(grid,result.result.path,self.args.radius) else None

    def refresh_home(self):
        path = self.plan(self.s.pose(),self.home)
        if path is None:
            self.reason = 'No verified home path'
            return False
        self.home_budget = route_seconds(path,self.args.return_speed)+60.
        self.home_checked = time.monotonic()
        return self.remaining()>self.home_budget+5.

    def move(self, goal=None, observe=False):
        if observe:
            if not self.s.spin.server_is_ready():
                return False
            request = Spin.Goal()
            request.target_yaw, request.time_allowance.sec = 2*math.pi, 15
            client = self.s.spin
        else:
            request = NavigateToPose.Goal()
            request.pose = copy.deepcopy(goal)
            request.pose.header.stamp = self.s.get_clock().now().to_msg()
            client = self.s.navigate
        self.active = self.submit(client,request)
        if self.active is None:
            return False
        result = self.active.get_result_async()
        end = time.monotonic()+(15. if observe else 60.)
        while rclpy.ok() and not result.done():
            if self.s.issues(perception=False):
                self.cancel()
                raise RuntimeError('Navigation sensors/TF lost')
            if self.state != 'RETURN':
                if self.s.issues() or self.remaining() <= self.home_budget+10.:
                    self.reason = 'Perception unavailable or return reserve reached'
                    self.cancel()
                    return False
                if time.monotonic()-self.home_checked >= 5.:
                    if not self.refresh_home():
                        self.cancel()
                        return False
            if time.monotonic()>end or self.remaining()<=0:
                self.cancel()
                return False
            self.checkpoint()
            self.publish()
            time.sleep(.05)
        self.active = None
        return result.done() and result.result().status == GoalStatus.STATUS_SUCCEEDED

    def checkpoint(self, final=False):
        if self.saving is not None:
            if not self.saving.done() and not final:
                return
            try:
                self.saving.result(timeout=10. if final else 0.)
                self.export_error = None
            except Exception as error:
                self.export_error = str(error)
                if not self.saving.done():
                    return
            self.saving = None
        if not final and time.monotonic()-self.last_save < 20.:
            return
        self.last_save = time.monotonic()
        with self.s.lock:
            grid = self.s.grid
            tags = dict(frame_id=self.args.map_frame,tags=summarize(self.s.records),
                        dropped_observations=self.s.dropped)
            raw = list(self.s.raw)
            self.s.raw.clear()
        result = dict(state=self.state,reason=self.reason,duration_sec=self.args.duration,
                      elapsed_sec=self.args.duration-self.remaining(),
                      returned_home=self.arrival is not None,home_arrival_elapsed_sec=self.arrival,
                      returned_before_deadline=self.arrival is not None and self.arrival<=self.args.duration,
                      expected_tag_count=self.args.expected_tags)
        def save():
            self.output.mkdir(parents=True,exist_ok=True)
            with (self.output/'observations.jsonl').open('a') as stream:
                for observation in raw:
                    stream.write(json.dumps(observation)+'\n')
            export(self.output,grid,tags,result)
        self.saving = self.writer.submit(save)
        if final:
            try:
                self.saving.result(timeout=10.)
                self.export_error = None
            except Exception as error:
                self.export_error = str(error)
                print(f'Export failed: {error}',file=sys.stderr)

    def run(self):
        end = time.monotonic()+self.args.ready_timeout
        while rclpy.ok():
            issues = self.s.issues()
            self.publish(issues)
            if not issues:
                break
            if time.monotonic()>end:
                raise RuntimeError('Readiness timeout: '+'; '.join(issues))
            time.sleep(.2)
        input(f'Robot ready. Press Enter to start the {self.args.duration:g}-second mission: ')
        issues = self.s.issues()
        if issues:
            raise RuntimeError('Readiness lost: '+'; '.join(issues))
        self.home = self.s.pose()
        self.started = time.monotonic()
        self.output.mkdir(parents=True,exist_ok=True)
        atomic(self.output/'home.yaml',json.dumps(dict(frame_id=self.args.map_frame,
               x=self.home.pose.position.x,y=self.home.pose.position.y,
               yaw=yaw(self.home.pose.orientation))).encode())
        with self.s.lock:
            self.s.recording = True
        empty = 0
        last_grid = None
        while self.remaining()>0:
            self.state = 'EXPLORE'
            if self.s.issues() or not self.refresh_home():
                self.reason = self.reason or 'Return reserve or perception health requires return'
                break
            with self.s.lock:
                grid = self.s.grid
            current = self.s.pose()
            candidates = frontiers(grid,current.pose.position,self.args.radius,time.monotonic()+2.)
            self.cooldown = [(x,y,t) for x,y,t in self.cooldown if t>time.monotonic()]
            selected = None
            until = time.monotonic()+5.
            for _,x,y,angle in candidates[:30]:
                if time.monotonic()>until or self.remaining()<self.home_budget+20.:
                    break
                if any(math.hypot(x-a,y-b)<.5 for a,b,_ in self.cooldown):
                    continue
                goal = copy.deepcopy(current)
                goal.pose.position.x,goal.pose.position.y = x,y
                goal.pose.orientation.x = goal.pose.orientation.y = 0.
                goal.pose.orientation.z,goal.pose.orientation.w = math.sin(angle/2),math.cos(angle/2)
                path = self.plan(current,goal)
                back = self.plan(goal,self.home) if path else None
                if back and self.remaining()>route_seconds(path,self.args.return_speed)+route_seconds(back,self.args.return_speed)+80.:
                    selected = goal
                    break
            if selected is None:
                if grid is not last_grid:
                    empty = empty+1 if not candidates else 0
                last_grid = grid
                if empty >= 3:
                    self.reason = 'No reachable frontiers in three map updates'
                    break
                self.checkpoint()
                self.publish()
                time.sleep(.2)
                continue
            empty = 0
            self.cooldown.append((selected.pose.position.x,selected.pose.position.y,time.monotonic()+60.))
            success = self.move(selected)
            if self.reason or not self.refresh_home():
                break
            if success and self.remaining()>self.home_budget+20.:
                self.state = 'OBSERVE'
                self.move(observe=True)
                if self.reason:
                    break
        self.state = 'RETURN'
        self.cancel()
        for _ in range(3):
            if self.verify_home():
                break
            if self.remaining()<=0 or self.s.issues(perception=False):
                break
            if self.plan(self.s.pose(),self.home):
                self.move(self.home)
            if self.verify_home():
                break
        self.state = 'FINISHED' if self.arrival is not None and self.arrival<=self.args.duration else 'FAILED'
        self.reason = self.reason or ('Returned home' if self.state=='FINISHED' else 'Return deadline missed')

    def verify_home(self):
        stable = None
        end = time.monotonic()+1.5
        while time.monotonic()<end:
            if self.s.issues(perception=False):
                return False
            pose = self.s.pose().pose.position
            with self.s.lock:
                odom = self.s.odom
            velocity = odom.twist.twist
            if (math.hypot(pose.x-self.home.pose.position.x,pose.y-self.home.pose.position.y)>.25 or
                    math.hypot(velocity.linear.x,velocity.linear.y)>.02 or abs(velocity.angular.z)>.05):
                return False
            stable = stable or time.monotonic()
            if time.monotonic()-stable>=.5:
                self.arrival = self.args.duration-self.remaining()
                return True
            time.sleep(.05)
        return False


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--duration',type=float,default=240.)
    parser.add_argument('--expected-tags',type=int,default=12)
    parser.add_argument('--output',default='~/challenge_results')
    parser.add_argument('--radius',type=float,default=.15)
    parser.add_argument('--return-speed',type=float,default=.10,help='Conservative timing estimate, not commanded speed')
    parser.add_argument('--ready-timeout',type=float,default=120.)
    for name,default in [('map-topic','/map'),('odom-topic','/odom'),('scan-topic','/scan'),
                         ('detections-topic','/camera/detections'),
                         ('camera-info-topic','/camera/camera/color/camera_info'),
                         ('map-frame','map'),('base-frame','base_link')]:
        parser.add_argument('--'+name,default=default)
    args = parser.parse_args(argv)
    for name in ('duration','radius','return_speed','ready_timeout'):
        if not math.isfinite(getattr(args,name)) or getattr(args,name)<=0:
            parser.error(name+' must be finite and positive')
    if args.expected_tags<0:
        parser.error('expected-tags must be nonnegative')
    return args


def main():
    args = arguments()
    rclpy.init(args=[])
    mission = None
    success = False
    try:
        mission = Mission(args)
        mission.run()
        success = mission.state == 'FINISHED'
    except (KeyboardInterrupt,EOFError,Exception) as error:
        print(f'Mission stopped: {error}',file=sys.stderr)
        if mission:
            mission.state,mission.reason = 'FAILED',str(error) or 'Operator interruption'
    finally:
        if mission:
            for operation in (mission.cancel, mission.drain_pending):
                try:
                    operation()
                except Exception as error:
                    success = False
                    mission.state,mission.reason = 'FAILED',f'Cancellation failed: {error}'
            if mission.started is not None:
                with mission.s.lock:
                    mission.s.recording = False
                flush_until = time.monotonic()+1.1
                while time.monotonic()<flush_until:
                    mission.s.process_tags()
                    with mission.s.lock:
                        if not mission.s.pending:
                            break
                    time.sleep(.05)
                mission.checkpoint(final=True)
                if mission.export_error:
                    mission.state,mission.reason = 'FAILED','Final export failed: '+mission.export_error
                success = success and mission.export_error is None
                mission.last_status = 0.
                mission.publish()
                print(f'Results: {mission.output}',flush=True)
            mission.writer.shutdown(wait=True)
            mission.s.stop()
        rclpy.try_shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
