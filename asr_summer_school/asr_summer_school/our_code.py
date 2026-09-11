#! /usr/bin/env python3

from collections import Counter, deque
import math
import threading
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from tf2_msgs.msg import TFMessage
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import tf2_py as tf2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray


"""
Explore eligible frontier goals using Nav2 planning, while mapping
AprilTags into the global frame. An empty local frontier list does not prove
that the entire environment has been explored.
"""

# Observe idle frontier conditions across multiple SLAM updates (5 seconds each).
# This interval starts only while idle with fresh data, never before navigation.
NO_FRONTIER_TIMEOUT = 15.0
GOAL_RADIUS = 0.30
MIN_GOAL_DISTANCE = 0.50
# Frontier cells are clustered by 8-connected adjacency; a cluster whose extent (cell
# count x resolution) is below this is ignored, so a single-cell gap next to the robot
# doesn't out-rank a real corridor just for being closer.
MIN_FRONTIER_SIZE = 0.0
SUCCESS_COOLDOWN = 30.0
FAILURE_COOLDOWN = 60.0

# Total time budget for exploring before heading back to the start, whether or not the
# maze was fully explored.
MISSION_TIME_LIMIT = 4 * 60.0
# Number of distinct AprilTags placed in the maze: once all are confirmed, head back
# immediately rather than continuing to explore for the rest of the time budget.
TARGET_TAG_COUNT = 12
# Estimating the return trip from straight-line distance alone underestimates actual
# corridor travel, so it's scaled up for a safety margin, plus a flat buffer on top.
RETURN_DISTANCE_FACTOR = 2.5
RETURN_SPEED_ESTIMATE = 0.15  # m/s, conservative average including turns
RETURN_TIME_MARGIN = 15.0  # s


class FrontierExplorer(Node):
    """Caches the latest frontier centroids published by frontier_detection_node."""

    def __init__(self, node_name='frontier_explorer', frontier_topic='/frontier_centroids'):
        super().__init__(node_name)

        self._lock = threading.Lock()
        self._frontiers = []
        self._frame_id = 'map'
        self._received = 0.0
        self._sequence = 0
        self._map = None
        self._map_received = 0.0
        self._map_version = 0
        self._search_cache = None
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pose', 1)
        self.create_timer(0.2, self._publish_pose)

        self.create_subscription(
            Marker, frontier_topic, self._frontier_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.create_subscription(
            OccupancyGrid, '/map', self._map_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._executor = None
        self._thread = None

    def _frontier_callback(self, msg):
        with self._lock:
            self._frontiers = [(p.x, p.y) for p in msg.points]
            self._frame_id = msg.header.frame_id or 'map'
            self._received = time.monotonic()
            self._sequence += 1

    def _map_callback(self, msg):
        with self._lock:
            self._map = msg
            self._map_received = time.monotonic()
            self._map_version += 1
            self._sequence += 1

    def map_snapshot(self):
        with self._lock:
            return self._map, self._map_received, self._map_version

    def search(self, grid, version, robot):
        geometry = GridGeometry(grid)
        key = (version, geometry.cell(robot))
        if self._search_cache is None or self._search_cache[0] != key:
            self._search_cache = (key, reachable_frontier_goals(grid, robot))
        return self._search_cache[1]

    def snapshot(self):
        with self._lock:
            return (list(self._frontiers), self._frame_id,
                    self._received, self._sequence)

    def robot_transform(self, frame):
        try:
            transform = self.buffer.lookup_transform(frame, 'base_link', Time())
            age = (self.get_clock().now().nanoseconds -
                   Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
            if not -0.5 <= age <= 2.0:
                return None
            return transform
        except tf2.TransformException:
            return None

    def robot_position(self, frame):
        transform = self.robot_transform(frame)
        if transform is None:
            return None
        t = transform.transform.translation
        return (t.x, t.y) if math.isfinite(t.x) and math.isfinite(t.y) else None

    def _publish_pose(self):
        transform = self.robot_transform('map')
        if transform is None:
            return
        pose = PoseWithCovarianceStamped()
        pose.header = transform.header
        t = transform.transform.translation
        pose.pose.pose.position.x = t.x
        pose.pose.pose.position.y = t.y
        pose.pose.pose.position.z = t.z
        pose.pose.pose.orientation = transform.transform.rotation
        self.pose_pub.publish(pose)

    def start(self):
        if self._thread is not None:
            return self
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        if self._executor is not None and self._thread is not None:
            self._executor.shutdown()
            self._thread.join(timeout=2.0)
            self._executor.remove_node(self)
            self._executor = None
            self._thread = None
        self.destroy_node()


class TagObservations:
    """Confirm and refresh a tag only from three distinct, consistent images."""

    def __init__(self):
        self.last_stamp = {}
        self.samples = {}

    def add(self, key, stamp, position):
        if stamp <= self.last_stamp.get(key, -1):
            return None
        self.last_stamp[key] = stamp
        if not all(math.isfinite(v) for v in position):
            self.samples.pop(key, None)
            return None
        samples = self.samples.get(key, [])
        if samples and (stamp-samples[0][0] > 2_000_000_000 or
                        any(math.dist(position, p) > 0.20 for _, p in samples)):
            samples = []
        samples.append((stamp, position))
        self.samples[key] = samples
        if len(samples) < 3:
            return None
        mean = tuple(sum(p[i] for _, p in samples)/3 for i in range(3))
        self.samples.pop(key)
        return mean


class AprilTagMapper(Node):
    """Looks up detected AprilTags in the map frame and publishes them as persistent markers."""

    def __init__(self, node_name='apriltag_mapper', detections_topic='/camera/detections',
                 marker_topic='/apriltag_markers', map_frame='map'):
        super().__init__(node_name)

        self.map_frame = map_frame
        self._lock = threading.RLock()
        self._found = {}
        self._observations = TagObservations()
        self._pending = deque()
        self._last_image_stamp = -1
        self._exact_tf = {}
        self._warnings = {}

        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)

        self.marker_pub = self.create_publisher(
            MarkerArray, marker_topic,
            QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            AprilTagDetectionArray, detections_topic, self._detections_callback, 10)
        # A TF lookup at a requested stamp may interpolate and report that stamp.
        # Observe raw TF messages too, to prove a tag TF existed for this image.
        self.create_subscription(TFMessage, '/tf', self._tf_callback, 100)
        self.create_timer(0.05, self._retry_pending)
        self._jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True),
            post_callback=self._clock_reset)

        self._executor = None
        self._thread = None

    def _detections_callback(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        with self._lock:
            if stamp <= 0 or stamp <= self._last_image_stamp:
                return
            self._last_image_stamp = stamp
            if not msg.detections:
                return
            if len(self._pending) >= 100:
                self._pending.popleft()
                self._warn('queue', 'Tag observation queue full; discarding oldest image.')
            keys = {(tag.family, tag.id) for tag in msg.detections}
            self._pending.append((stamp, msg.header.frame_id, keys, time.monotonic()+0.5))

    def _warn(self, key, message):
        now = time.monotonic()
        if now-self._warnings.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self._warnings[key] = now

    def _tf_callback(self, msg):
        with self._lock:
            now = time.monotonic()
            for tf in msg.transforms:
                if ':' in tf.child_frame_id:
                    stamp = Time.from_msg(tf.header.stamp).nanoseconds
                    self._exact_tf[(tf.header.frame_id, tf.child_frame_id, stamp)] = now
            self._prune_tf(now)

    def _prune_tf(self, now):
        self._exact_tf = {k: t for k, t in self._exact_tf.items() if now-t <= 1.0}

    def _retry_pending(self):
        with self._lock:
            now = time.monotonic()
            self._prune_tf(now)
            remaining = deque()
            blocked_keys = set()
            updated = False
            for stamp, parent, keys, deadline in self._pending:
                if now >= deadline:
                    self._warn('tf', 'Discarding tag observation: matching image-time TF unavailable within 0.5 s.')
                    continue
                waiting = set()
                for key in sorted(keys):
                    child = f'{key[0]}:{key[1]}'
                    if key in blocked_keys or (parent, child, stamp) not in self._exact_tf:
                        waiting.add(key)
                        continue
                    try:
                        tf = self.buffer.lookup_transform(
                            self.map_frame, child, Time(nanoseconds=stamp))
                    except tf2.TransformException:
                        waiting.add(key)
                        continue
                    t = tf.transform.translation
                    position = self._observations.add(key, stamp, (t.x, t.y, t.z))
                    if position is None:
                        continue
                    marker = Marker()
                    marker.header.frame_id = self.map_frame
                    marker.header.stamp = Time(nanoseconds=stamp).to_msg()
                    marker.ns = 'apriltags/'+key[0]
                    marker.id = key[1]
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.25
                    marker.color.a = marker.color.g = 1.0

                    label = Marker()
                    label.header.frame_id = self.map_frame
                    label.header.stamp = marker.header.stamp
                    label.ns = 'apriltag_labels/'+key[0]
                    label.id = key[1]
                    label.type = Marker.TEXT_VIEW_FACING
                    label.action = Marker.ADD
                    label.text = str(key[1])
                    label.pose.position.x, label.pose.position.y = position[0], position[1]
                    label.pose.position.z = position[2] + 0.3
                    label.pose.orientation.w = 1.0
                    label.scale.z = 0.2
                    label.color.a = label.color.r = label.color.g = label.color.b = 1.0

                    first = key not in self._found
                    self._found[key] = (marker, label)
                    updated = True
                    if first:
                        self.get_logger().info(f'Confirmed AprilTag {child}; found so far: {self.found_ids}')
                if waiting:
                    remaining.append((stamp, parent, waiting, deadline))
                    blocked_keys.update(waiting)
            self._pending = remaining
            if updated:
                self._publish_markers()

    def _clock_reset(self, _jump):
        with self._lock:
            deletions = []
            for pair in self._found.values():
                for marker in pair:
                    deletion = Marker()
                    deletion.header.frame_id = self.map_frame
                    deletion.ns, deletion.id = marker.ns, marker.id
                    deletion.action = Marker.DELETE
                    deletions.append(deletion)
            self._found.clear()
            self._pending.clear()
            self._exact_tf.clear()
            self._observations = TagObservations()
            self._last_image_stamp = -1
            if deletions:
                self.marker_pub.publish(MarkerArray(markers=deletions))
            self.get_logger().info('ROS clock changed: cleared tag confirmations and pending observations.')

    def _publish_markers(self):
        with self._lock:
            markers = [marker for pair in self._found.values() for marker in pair]
        self.marker_pub.publish(MarkerArray(markers=markers))

    @property
    def found_ids(self):
        with self._lock:
            return sorted(f'{family}:{tag_id}' for family, tag_id in self._found)

    def start(self):
        if self._thread is not None:
            return self
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        if self._executor is not None and self._thread is not None:
            self._executor.shutdown()
            self._thread.join(timeout=2.0)
            self._executor.remove_node(self)
            self._executor = None
            self._thread = None
        self.destroy_node()


class GridGeometry:
    """Conversions and conservative segment traversal, including rotated maps."""

    def __init__(self, grid):
        self.grid = grid
        self.w, self.h, self.res = grid.info.width, grid.info.height, grid.info.resolution
        self.origin = grid.info.origin.position
        q = grid.info.origin.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        self.c, self.s = math.cos(yaw), math.sin(yaw)

    def valid(self):
        return (self.w > 0 and self.h > 0 and math.isfinite(self.res)
                and self.res > 0 and len(self.grid.data) == self.w*self.h
                and all(math.isfinite(v) for v in (self.origin.x,self.origin.y,self.c,self.s)))

    def local(self, point):
        dx, dy = point[0]-self.origin.x, point[1]-self.origin.y
        return ((self.c*dx+self.s*dy)/self.res, (-self.s*dx+self.c*dy)/self.res)

    def cell(self, point):
        return tuple(math.floor(v) for v in self.local(point))

    def inside(self, cell):
        x, y = cell
        return 0 <= x < self.w and 0 <= y < self.h

    def value(self, cell):
        return self.grid.data[cell[1]*self.w+cell[0]]

    def world(self, cell):
        x, y = (cell[0]+0.5)*self.res, (cell[1]+0.5)*self.res
        return (self.origin.x+self.c*x-self.s*y, self.origin.y+self.s*x+self.c*y)

    def neighbors(self, cell):
        x, y = cell
        return [p for p in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)) if self.inside(p)]

    def segment_cells(self, start, end):
        # Split at every grid-line crossing. At exact boundaries check both
        # adjacent cells, including all four cells at a corner.
        ax, ay = self.local(start)
        bx, by = self.local(end)
        times = {0.0, 1.0}
        for a, b in ((ax,bx),(ay,by)):
            if abs(b-a) > 1e-12:
                for boundary in range(math.floor(min(a,b))+1, math.ceil(max(a,b))):
                    times.add((boundary-a)/(b-a))
        ordered = sorted(times)
        samples = ordered + [(a+b)/2 for a,b in zip(ordered, ordered[1:])]
        cells = set()
        for t in samples:
            x, y = ax+t*(bx-ax), ay+t*(by-ay)
            xs = {math.floor(x)}
            ys = {math.floor(y)}
            if abs(x-round(x)) < 1e-9:
                xs.update((round(x)-1, round(x)))
            if abs(y-round(y)) < 1e-9:
                ys.update((round(y)-1, round(y)))
            cells.update((ix,iy) for ix in xs for iy in ys)
        return cells


def reachable_frontier_goals(grid, robot):
    """Search entire seeded free components; nearby seeds need Nav2 validation."""
    g = GridGeometry(grid)
    stats = dict(state='invalid map', total_free=0, connected=0, components=0,
                 frontier_cells=0, clearance_rejected=0, frontier_clusters=0,
                 small_frontier_rejected=0, seed_distance=None)
    if not g.valid() or not all(math.isfinite(v) for v in robot):
        return [], stats
    stats['total_free'] = grid.data.count(0)
    cell = g.cell(robot)
    if not g.inside(cell):
        stats['state'] = 'robot outside map'
        return [], stats
    value = g.value(cell)
    stats['state'] = 'robot free' if value == 0 else 'robot unknown' if value == -1 else 'robot occupied'
    if value > 0:
        return [], stats
    if value == 0:
        seeds = [cell]
    else:
        radius = math.ceil(GOAL_RADIUS/g.res)
        seeds = [(x,y) for y in range(max(0,cell[1]-radius), min(g.h,cell[1]+radius+1))
                 for x in range(max(0,cell[0]-radius), min(g.w,cell[0]+radius+1))
                 if g.value((x,y)) == 0 and math.dist(g.world((x,y)),robot) <= GOAL_RADIUS]
        seeds.sort(key=lambda p: (math.dist(g.world(p),robot), p[1]*g.w+p[0]))
    if not seeds:
        stats['state'] += '; no free seed within 0.30 m'
        return [], stats
    stats['seed_distance'] = math.dist(g.world(seeds[0]), robot)
    visited, candidates = set(), []
    radius = math.ceil(0.20/g.res)
    offsets = [(dx,dy) for dy in range(-radius,radius+1) for dx in range(-radius,radius+1)
               if (dx*dx+dy*dy)*g.res*g.res <= 0.20**2]
    for seed in seeds:
        if seed in visited:
            continue
        stats['components'] += 1
        visited.add(seed)
        queue = deque([(seed, math.dist(g.world(seed),robot))])
        while queue:
            current, distance = queue.popleft()
            adjacent_unknown = False
            for neighbor in g.neighbors(current):
                value = g.value(neighbor)
                adjacent_unknown |= value == -1
                if value == 0 and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, distance+g.res))
            if not adjacent_unknown:
                continue
            stats['frontier_cells'] += 1
            x, y = current
            if any(g.inside((x+dx,y+dy)) and g.value((x+dx,y+dy)) > 0 for dx,dy in offsets):
                stats['clearance_rejected'] += 1
                continue
            candidates.append((distance,current,g.world(current)))
    stats['connected'] = len(visited)

    # Cluster frontier cells by 8-connected adjacency so each candidate can be judged
    # by its cluster's physical extent, not in isolation: a single-cell gap shouldn't
    # out-rank a real corridor opening just for being closer to the robot.
    frontier_cells = {cell for _,cell,_ in candidates}
    cluster_of = {}
    cluster_sizes = []
    for start in frontier_cells:
        if start in cluster_of:
            continue
        index = len(cluster_sizes)
        cluster_of[start] = index
        stack, size = [start], 0
        while stack:
            cx, cy = stack.pop()
            size += 1
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    neighbor = (cx+dx,cy+dy)
                    if (dx or dy) and neighbor in frontier_cells and neighbor not in cluster_of:
                        cluster_of[neighbor] = index
                        stack.append(neighbor)
        cluster_sizes.append(size)
    stats['frontier_clusters'] = len(cluster_sizes)

    before_size_filter = len(candidates)
    candidates = [c for c in candidates if cluster_sizes[cluster_of[c[1]]]*g.res >= MIN_FRONTIER_SIZE]
    stats['small_frontier_rejected'] = before_size_filter-len(candidates)

    selected = []
    for _, _, point in sorted(candidates):
        if math.dist(point,robot) <= GOAL_RADIUS:
            continue
        if all(math.dist(point,other) >= 0.40 for other in selected):
            selected.append(point)
    return selected, stats


def validate_path(path, robot, target, frame):
    """Check result sanity and progress; Nav2 owns collision/traversability checks."""
    if not path.poses or path.header.frame_id != frame:
        return 'empty path or unexpected path frame'
    points = []
    for pose in path.poses:
        p, q = pose.pose.position, pose.pose.orientation
        if pose.header.frame_id not in ('', frame) or not all(
                math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
            return 'nonfinite path or unexpected pose frame'
        points.append((p.x,p.y))
    if math.dist(points[-1],target) > GOAL_RADIUS:
        return 'path endpoint misses requested goal'
    if math.dist(points[-1],robot) <= GOAL_RADIUS:
        return 'path endpoint is already reached'
    return None


class PathValidator:
    """A separate action client keeps planning results out of navigation state."""

    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        self.pending = None
        self.handle = None
        self.validated_path = None

    def wait(self, future, deadline):
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=min(0.1, max(0.0, deadline-time.monotonic())))
        return future.done()

    @staticmethod
    def cancel_late(future):
        if not future.cancelled() and future.exception() is None:
            handle = future.result()
            if handle.accepted:
                handle.cancel_goal_async()

    def cancel(self):
        if self.handle is not None:
            self.handle.cancel_goal_async()
            self.handle = None
        elif self.pending is not None:
            self.pending.add_done_callback(self.cancel_late)
        self.pending = None

    def check(self, goal, robot):
        self.validated_path = None
        if not self.client.wait_for_server(timeout_sec=5.0):
            return 'unavailable', 'planner action server unavailable'
        deadline = time.monotonic()+5.0
        request = ComputePathToPose.Goal()
        request.goal = goal
        request.planner_id = 'GridBased'
        request.use_start = False
        try:
            self.pending = self.client.send_goal_async(request)
            if not self.wait(self.pending,deadline):
                self.cancel()
                return 'unavailable', 'planner goal acknowledgement timed out'
            self.handle = self.pending.result()
            self.pending = None
            if not self.handle.accepted:
                self.handle = None
                return 'invalid', 'planner rejected goal'
            result_future = self.handle.get_result_async()
            if not self.wait(result_future,deadline):
                self.cancel()
                return 'unavailable', 'planner result timed out'
            result = result_future.result()
            self.handle = None
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                return 'invalid', 'planner failed to find a path'
            reason = validate_path(result.result.path,robot,
                                   (goal.pose.position.x,goal.pose.position.y),goal.header.frame_id)
            if reason is None:
                self.validated_path = result.result.path
            return ('invalid',reason) if reason else ('valid','path validated')
        except Exception as error:
            self.cancel()
            return 'unavailable', f'planner communication error: {error}'


class GoalPolicy:
    """Spatial cooldowns and idle timers, driven exclusively by simulation time."""

    def __init__(self):
        self.attempts = []
        self.idle_kind = None
        self.idle_since = None
        self.last_time = None

    def reset_idle(self):
        self.idle_kind = self.idle_since = None

    def tick(self, now):
        if self.last_time is not None and now < self.last_time:
            # A simulation reset invalidates old locations and timer baselines.
            self.attempts.clear()
            self.reset_idle()
        self.last_time = now
        self.attempts = [a for a in self.attempts if a[2] > now]

    def record(self, goal, frame, now, success):
        self.attempts.append((goal, frame, now + (
            SUCCESS_COOLDOWN if success else FAILURE_COOLDOWN)))
        self.reset_idle()

    def candidates(self, points, frame, robot):
        eligible = []
        suppressed = False
        for point in points:
            if not all(math.isfinite(v) for v in point):
                continue
            if math.dist(point, robot) < MIN_GOAL_DISTANCE:
                continue
            if any(f == frame and math.dist(point, g) <= GOAL_RADIUS
                   for g, f, _ in self.attempts):
                suppressed = True
                continue
            eligible.append(point)
        return sorted(eligible, key=lambda p: math.dist(p, robot)), suppressed

    def idle_expired(self, kind, now):
        if self.idle_kind != kind:
            self.idle_kind, self.idle_since = kind, now
        return now - self.idle_since >= NO_FRONTIER_TIMEOUT


def rank_candidates(points, robot, heading):
    """Prefer smaller turns among goals within 25 cm of the nearest distance."""
    remaining = sorted(set(points), key=lambda p: (math.dist(p, robot), p))
    ranked = []
    while remaining:
        limit = math.dist(remaining[0], robot) + 0.25
        group = [p for p in remaining if math.dist(p, robot) <= limit]
        def key(point):
            angle = math.atan2(point[1]-robot[1], point[0]-robot[0])-heading
            return abs(math.atan2(math.sin(angle), math.cos(angle))), math.dist(point, robot), point
        ranked.extend(sorted(group, key=key))
        remaining = remaining[len(group):]
    return ranked


def path_final_heading(path):
    for first, last in reversed(list(zip(path.poses, path.poses[1:]))):
        a, b = first.pose.position, last.pose.position
        if math.hypot(b.x-a.x, b.y-a.y) > 1e-6:
            return math.atan2(b.y-a.y, b.x-a.x)
    return None


def make_goal_pose(navigator, frame_id, x, y, robot, yaw=None):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = frame_id
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    if yaw is None:
        yaw = math.atan2(y - robot[1], x - robot[0])
    goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
    return goal_pose


def wait_for_task(navigator, should_abort=None):
    """Block until the current task ends, cancelling early if `should_abort()` fires.

    Used so a frontier approach already in flight gets interrupted the moment it would
    eat into the reserved return-to-start time, rather than only checking the budget
    between goals.
    """
    aborted = False
    while rclpy.ok():
        if navigator.isTaskComplete():
            return navigator.getResult()
        if not aborted and should_abort is not None and should_abort():
            navigator.cancelTask()
            aborted = True
        time.sleep(0.05)
    return TaskResult.CANCELED


def estimate_return_seconds(robot, home):
    """Conservative estimate of how long driving from `robot` back to `home` would take."""
    return RETURN_DISTANCE_FACTOR * math.dist(robot, home) / RETURN_SPEED_ESTIMATE + RETURN_TIME_MARGIN


def relax_yaw_tolerance(navigator, yaw_tolerance=math.pi):
    """Make controller_server accept any final heading.

    Only used for the return-to-start leg: we only care about reaching the starting
    (x, y), not which way the robot ends up facing. `general_goal_checker` is the
    goal_checker plugin name from param_nav2.yaml.
    """
    client = navigator.create_client(SetParameters, '/controller_server/set_parameters')
    if not client.wait_for_service(timeout_sec=5.0):
        navigator.get_logger().warning(
            'controller_server/set_parameters unavailable; final orientation will '
            'still be checked.')
        navigator.destroy_client(client)
        return
    request = SetParameters.Request()
    request.parameters = [ParameterMsg(
        name='general_goal_checker.yaw_goal_tolerance',
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=yaw_tolerance))]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=5.0)
    navigator.destroy_client(client)


def return_home(navigator, frontiers, home_position):
    """Drive back to `home_position`; only the final (x, y) matters, not heading."""
    navigator.get_logger().info(
        f'Heading back to the starting point at x={home_position[0]:.2f}, '
        f'y={home_position[1]:.2f}.')
    relax_yaw_tolerance(navigator)
    goal = make_goal_pose(navigator, 'map', *home_position, home_position, yaw=0.0)
    accepted = navigator.goToPose(goal)
    result = wait_for_task(navigator) if accepted else TaskResult.FAILED
    end = frontiers.robot_position('map')
    success = (result == TaskResult.SUCCEEDED and end is not None
               and math.dist(end, home_position) <= GOAL_RADIUS)
    if success:
        navigator.get_logger().info('Back at the starting point.')
    else:
        navigator.get_logger().warning('Could not fully return to the starting point.')
    return success


def explore(navigator, frontiers, validator, apriltags, home_position, mission_deadline):
    policy = GoalPolicy()
    required_sequence = 0
    reports = {}
    rejections = Counter()

    def report(message, category='status'):
        previous, when = reports.get(category, (None, 0.0))
        now = time.monotonic()
        if (message != previous and now-when >= 2.0) or now-when >= 15.0:
            navigator.get_logger().info(message)
            reports[category] = (message, now)

    while rclpy.ok():
        now = frontiers.get_clock().now().nanoseconds/1e9
        clock_reset = policy.last_time is not None and now < policy.last_time
        policy.tick(now)
        points, point_frame, received, sequence = frontiers.snapshot()
        grid, map_received, version = frontiers.map_snapshot()
        if clock_reset:
            required_sequence = sequence+1
        if grid is None or time.monotonic()-map_received > NO_FRONTIER_TIMEOUT:
            policy.reset_idle()
            report('Waiting: SLAM map missing or stale; not an empty-frontier result.')
            time.sleep(0.1)
            continue
        frame = grid.header.frame_id or 'map'
        robot = frontiers.robot_position(frame)
        if robot is None:
            policy.reset_idle()
            report('Waiting: robot TF missing or stale.')
            time.sleep(0.1)
            continue
        if len(apriltags.found_ids) >= TARGET_TAG_COUNT:
            navigator.get_logger().info(
                f'All {TARGET_TAG_COUNT} AprilTags found: heading back to the starting point.')
            return
        if now >= mission_deadline - estimate_return_seconds(robot, home_position):
            navigator.get_logger().info(
                'Mission time budget reached: heading back to the starting point.')
            return
        if sequence < required_sequence:
            policy.reset_idle()
            report('Waiting for a map or frontier update after the last action.')
            time.sleep(0.1)
            continue
        if not GridGeometry(grid).valid():
            policy.reset_idle()
            report('Waiting: invalid occupancy grid.')
            time.sleep(0.1)
            continue
        alternatives, stats = frontiers.search(grid,version,robot)
        disconnected = stats['total_free']-stats['connected']
        seed = stats['seed_distance']
        seed_text = 'none' if seed is None else f'{seed:.2f} m'
        report(f"Search: {stats['state']}; seed={seed_text}; free connected/total="
               f"{stats['connected']}/{stats['total_free']}; components={stats['components']}; "
               f"frontier cells={stats['frontier_cells']}; clearance rejected="
               f"{stats['clearance_rejected']}; clusters={stats['frontier_clusters']}; "
               f"small frontier rejected={stats['small_frontier_rejected']}; "
               f"sampled candidates={len(alternatives)}", 'search')
        if stats['connected'] and disconnected:
            report(f'Disconnected mapped free space: {disconnected} cells outside seeded components; '
                   'physical reachability is not established.', 'disconnected')
        if not stats['connected']:
            policy.reset_idle()
            report(f"Waiting: search initialization failed ({stats['state']}); not exploration completion.")
            time.sleep(0.2)
            continue

        # Both sources compete on distance and heading, without source priority.
        supplied = points if point_frame == frame and time.monotonic()-received <= NO_FRONTIER_TIMEOUT else []
        eligible, suppressed = policy.candidates(supplied,frame,robot)
        recovered, cooling = policy.candidates(alternatives,frame,robot)
        transform = frontiers.robot_transform(frame)
        if transform is None:
            policy.reset_idle()
            time.sleep(0.1)
            continue
        q = transform.transform.rotation
        heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        candidates = rank_candidates(eligible + recovered, robot, heading)
        suppressed |= cooling
        report(f'Goals: supplied={len(supplied)}, map-derived={len(alternatives)}, '
               f'eligible={len(candidates)}, proximity/invalid/cooldown filtered='
               f'{len(supplied)+len(alternatives)-len(eligible)-len(recovered)}, '
               f'path rejection counts={dict(rejections)}', 'goals')
        target = None
        goal_heading = None
        infrastructure_failure = False
        changed = False
        for candidate in candidates[:8]:
            goal = make_goal_pose(navigator,frame,*candidate,robot)
            outcome, reason = validator.check(goal,robot)
            if frontiers.map_snapshot()[2] != version:
                report('Map changed during planning; discarding result and reassessing.', 'planning')
                changed = True
                break
            if outcome == 'unavailable':
                report(f'Waiting: {reason}; candidates are not blacklisted.', 'planning')
                infrastructure_failure = True
                break
            if outcome == 'valid':
                goal_heading = path_final_heading(validator.validated_path)
                if goal_heading is None:
                    rejections['path has no translational segment'] += 1
                    policy.record(candidate,frame,frontiers.get_clock().now().nanoseconds/1e9,False)
                    continue
                target = candidate
                break
            rejections[reason] += 1
            report(f'Path rejected for ({candidate[0]:.2f}, {candidate[1]:.2f}): {reason}', 'planning:'+reason)
            policy.record(candidate,frame,frontiers.get_clock().now().nanoseconds/1e9,False)
        if changed or infrastructure_failure:
            policy.reset_idle()
            time.sleep(0.2)
            continue
        if target is not None:
            # Refresh robot TF after planning; recheck the starting position before
            # sending NavigateToPose if the robot moved during validation.
            current = frontiers.robot_position(frame)
            if current is None or math.dist(current,robot)>0.05:
                report('Robot moved or TF expired during planning; reassessing.')
                time.sleep(0.1)
                continue
            policy.reset_idle()
            source = 'frontier' if target in eligible else 'map-derived frontier'
            navigator.get_logger().info(
                f'Heading to validated {source} at x={target[0]:.2f}, y={target[1]:.2f}')
            accepted = navigator.goToPose(make_goal_pose(navigator,frame,*target,robot,yaw=goal_heading))
            def time_to_go_home():
                if len(apriltags.found_ids) >= TARGET_TAG_COUNT:
                    return True
                current = frontiers.robot_position(frame) or robot
                now = frontiers.get_clock().now().nanoseconds/1e9
                return now >= mission_deadline - estimate_return_seconds(current, home_position)
            result = wait_for_task(navigator, should_abort=time_to_go_home) if accepted else TaskResult.FAILED
            end = frontiers.robot_position(frame)
            success = (result == TaskResult.SUCCEEDED and end is not None
                       and math.dist(end,target) <= GOAL_RADIUS)
            if success:
                navigator.get_logger().info(
                    f'Reached frontier; TF displacement={math.dist(robot,end):.2f} m.')
            elif result == TaskResult.SUCCEEDED:
                report('Nav2 success without TF-confirmed arrival; cooling down goal.')
            else:
                report('Navigation rejected or unsuccessful; cooling down goal.')
            now = frontiers.get_clock().now().nanoseconds/1e9
            policy.tick(now)
            if success:
                policy.record(robot,frame,now,True)
            policy.record(target,frame,now,success)
            required_sequence = frontiers.snapshot()[3]+1
            continue
        if candidates:
            # Failed candidates now have cooldowns. Next pass continues the rest
            # of this ranked list, including batches beyond the first eight.
            policy.reset_idle()
            time.sleep(0.1)
            continue

        kind = 'empty' if stats['frontier_cells']==0 and not supplied else 'filtered'
        now = frontiers.get_clock().now().nanoseconds/1e9
        expired = policy.idle_expired(kind,now)
        report('No frontiers in searched free-space components.' if kind=='empty'
               else 'Waiting: frontier candidates filtered or rejected by path validation.')
        if not expired:
            time.sleep(0.1)
            continue
        if suppressed:
            report('Waiting for cooldown expiry and fresh data; rejected goals are not completion.')
        else:
            report('Waiting for fresh data: no frontiers in searched free-space components.'
                   if kind=='empty' else
                   'Waiting for fresh data: all frontier candidates fail proximity or clearance checks.')
        # Idle exploration never issues velocity or spin commands. Resume when
        # new frontier/map data arrive, retaining tag processing in the meantime.
        required_sequence = sequence+1
        time.sleep(0.1)


def wait_for_navigation(navigator):
    """Retry lost lifecycle replies instead of hanging on a single ROS request."""
    for name in ('controller_server', 'bt_navigator'):
        client = navigator.create_client(GetState, f'/{name}/get_state')
        try:
            while rclpy.ok():
                if not client.wait_for_service(timeout_sec=5.0):
                    navigator.get_logger().info(f'Waiting for {name} lifecycle service.')
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(navigator, future, timeout_sec=5.0)
                if not future.done():
                    client.remove_pending_request(future)
                    future.cancel()
                    navigator.get_logger().warning(f'{name} lifecycle reply timed out; retrying.')
                    continue
                response = future.result()
                if response is not None and response.current_state.label == 'active':
                    break
                navigator.get_logger().info(f'Waiting for {name} to become active.')
                time.sleep(1.0)
        finally:
            navigator.destroy_client(client)
    navigator.get_logger().info('Nav2 is ready for use!')


def wait_for_home_position(frontiers, timeout=15.0):
    """Block until the robot's own TF-based position is available in the map frame, so
    there's somewhere to return to once the mission's time budget runs out."""
    deadline = time.monotonic() + timeout
    while True:
        position = frontiers.robot_position('map')
        if position is not None:
            return position
        if time.monotonic() > deadline:
            raise RuntimeError(f'No robot position in map frame after {timeout}s')
        time.sleep(0.2)


def main():
    # Keep the context alive during Ctrl+C so our finally block can cancel
    # the current action and join executor threads before shutting ROS down.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    navigator = frontiers = apriltags = validator = None
    try:
        navigator = BasicNavigator()
        frontiers = FrontierExplorer()
        apriltags = AprilTagMapper()
        validator = PathValidator(navigator)
        # Finish construction before starting background executors.
        frontiers.start()
        apriltags.start()
        wait_for_navigation(navigator)
        home_position = wait_for_home_position(frontiers)
        mission_deadline = frontiers.get_clock().now().nanoseconds/1e9 + MISSION_TIME_LIMIT
        explore(navigator, frontiers, validator, apriltags, home_position, mission_deadline)
        return_home(navigator, frontiers, home_position)
    except KeyboardInterrupt:
        pass
    finally:
        if validator is not None:
            validator.cancel()
        # Cancel our own outstanding action, without shutting down shared Nav2.
        if rclpy.ok() and navigator is not None and navigator.result_future is not None:
            if not navigator.result_future.done():
                navigator.cancelTask()
        if apriltags is not None:
            print(f'AprilTags found: {apriltags.found_ids}')
            apriltags.stop()
        if frontiers is not None:
            frontiers.stop()
        if navigator is not None:
            navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
