#! /usr/bin/env python3

import threading
import time

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
import tf2_py as tf2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from asr_summer_school.sensor_monitor import SensorMonitor

"""
Explore the maze by driving to frontier centroids (published by
frontier_detection_node), mapping every AprilTag seen along the way onto the
global map, until all tags are found or the mission time budget runs out --
then return to the starting point.
"""

# How long (s) the frontier list has to stay empty before we call exploration done.
# Must cover at least one slam_toolbox map_update_interval (5s, see param_slam_toolbox.yaml)
# so we don't quit right between two map updates.
NO_FRONTIER_TIMEOUT = 15.0

# Total time budget (s) for finding AprilTags before heading back to the start,
# whether or not all of them were found.
MISSION_TIME_LIMIT = 4 * 60.0

# Number of distinct AprilTags placed in the maze.
TARGET_TAG_COUNT = 12

# Robot frame used to look up the starting pose in the map, and to return to it later.
# Matches `robot_base_frame` in param_nav2.yaml.
ROBOT_FRAME = 'base_link'


class FrontierExplorer(Node):
    """Caches the latest frontier centroids published by frontier_detection_node."""

    def __init__(self, node_name='frontier_explorer', frontier_topic='/frontier_centroids'):
        super().__init__(node_name)

        self._lock = threading.Lock()
        self._frontiers = []
        self._frame_id = 'map'

        self.create_subscription(
            Marker, frontier_topic, self._frontier_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._executor = None
        self._thread = None

    def _frontier_callback(self, msg):
        with self._lock:
            self._frontiers = [(p.x, p.y) for p in msg.points]
            self._frame_id = msg.header.frame_id or 'map'

    @property
    def frontiers(self):
        """Latest list of (x, y) frontier centroids, in `frame_id`."""
        with self._lock:
            return list(self._frontiers)

    @property
    def frame_id(self):
        with self._lock:
            return self._frame_id

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


class AprilTagMapper(Node):
    """Looks up detected AprilTags in the map frame and publishes them as persistent markers."""

    def __init__(self, node_name='apriltag_mapper', detections_topic='/camera/detections',
                 marker_topic='/apriltag_markers', map_frame='map'):
        super().__init__(node_name)

        self.map_frame = map_frame
        self._lock = threading.Lock()
        self._found = {}

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.marker_pub = self.create_publisher(
            MarkerArray, marker_topic,
            QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            AprilTagDetectionArray, detections_topic, self._detections_callback, 10)

        self._executor = None
        self._thread = None

    def _detections_callback(self, msg):
        new_tag_found = False
        for tag in msg.detections:
            target_frame = f'{tag.family}:{tag.id}'
            try:
                tf = self.buffer.lookup_transform(self.map_frame, target_frame, Time())
            except tf2.TransformException:
                continue

            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'apriltags'
            marker.id = tag.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = tf.transform.translation.x
            marker.pose.position.y = tf.transform.translation.y
            marker.pose.position.z = tf.transform.translation.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.g = 1.0

            with self._lock:
                new_tag_found = new_tag_found or tag.id not in self._found
                self._found[tag.id] = marker

        if new_tag_found:
            self._publish_markers()
            self.get_logger().info(f'AprilTags found so far: {self.found_ids}')

    def _publish_markers(self):
        with self._lock:
            markers = list(self._found.values())
        self.marker_pub.publish(MarkerArray(markers=markers))

    @property
    def found_ids(self):
        with self._lock:
            return sorted(self._found.keys())

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


def make_goal_pose(navigator, frame_id, x, y):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = frame_id
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation.w = 1.0
    return goal_pose


def wait_for_start_pose(navigator, buffer, map_frame, robot_frame, timeout=15.0):
    """Block until `map_frame` -> `robot_frame` is available and return it as a PoseStamped.

    Captured once at startup, before any navigation, so we have somewhere to come back to.
    """
    deadline = time.time() + timeout
    while True:
        try:
            tf = buffer.lookup_transform(map_frame, robot_frame, Time())
            pose = PoseStamped()
            pose.header.frame_id = map_frame
            pose.header.stamp = navigator.get_clock().now().to_msg()
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            return pose
        except tf2.TransformException:
            if time.time() > deadline:
                raise RuntimeError(
                    f'No {map_frame} -> {robot_frame} transform after {timeout}s')
            time.sleep(0.2)


def navigate_to(navigator, goal_pose, should_abort=None):
    """Drive to `goal_pose`, calling `should_abort()` (if given) to cancel early.

    Returns the resulting TaskResult. A rejected goal request is reported as FAILED
    right away instead of falling through to isTaskComplete()/getResult(), which would
    otherwise replay the *previous* goal's already-resolved result -- goToPose() leaves
    result_future/status untouched when a goal is rejected, so isTaskComplete() finds a
    stale, already-done future and returns True instantly, without the robot moving.
    """
    if not navigator.goToPose(goal_pose):
        return TaskResult.FAILED

    i = 0
    while not navigator.isTaskComplete():
        if should_abort is not None and should_abort():
            navigator.cancelTask()
            break
        # AprilTag detection and marker publishing keep running in the background.
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    return navigator.getResult()


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Background nodes: they spin on their own, the main loop just reads their properties.
    sensors = SensorMonitor().start()
    frontiers = FrontierExplorer().start()
    apriltags = AprilTagMapper().start()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer='controller_server')

    # Remember where we started so we can come back here, tag hunt done or not.
    print('Looking up starting pose...')
    home_pose = wait_for_start_pose(navigator, apriltags.buffer, apriltags.map_frame, ROBOT_FRAME)
    print(f'Start pose: x={home_pose.pose.position.x:.2f}, y={home_pose.pose.position.y:.2f}')

    mission_deadline = time.time() + MISSION_TIME_LIMIT

    def mission_done():
        return len(apriltags.found_ids) >= TARGET_TAG_COUNT or time.time() >= mission_deadline

    # Frontiers that navigation failed to reach: skip them on future picks.
    failed_frontiers = set()

    def frontier_key(point):
        return (round(point[0], 1), round(point[1], 1))

    last_frontier_time = time.time()

    while rclpy.ok():
        if len(apriltags.found_ids) >= TARGET_TAG_COUNT:
            print(f'Found all {TARGET_TAG_COUNT} AprilTags!')
            break
        if time.time() >= mission_deadline:
            print(f'Mission time budget ({MISSION_TIME_LIMIT / 60:.0f} min) reached.')
            break

        candidates = [f for f in frontiers.frontiers if frontier_key(f) not in failed_frontiers]

        if not candidates:
            if time.time() - last_frontier_time > NO_FRONTIER_TIMEOUT:
                print('No more frontiers left: exploration complete.')
                break
            time.sleep(1.0)
            continue

        last_frontier_time = time.time()

        # Go to the frontier closest to the robot's current position.
        robot_x, robot_y = 0.0, 0.0
        if sensors.odom is not None:
            robot_x = sensors.odom.pose.pose.position.x
            robot_y = sensors.odom.pose.pose.position.y
        target_x, target_y = min(
            candidates, key=lambda p: (p[0] - robot_x) ** 2 + (p[1] - robot_y) ** 2)

        goal_pose = make_goal_pose(navigator, frontiers.frame_id, target_x, target_y)
        print(f'Heading to frontier at x={target_x:.2f}, y={target_y:.2f}')
        result = navigate_to(navigator, goal_pose, should_abort=mission_done)

        if result == TaskResult.SUCCEEDED:
            print('Reached frontier.')
            # If the frontier detector still republishes this same centroid after we've
            # reached it (e.g. a spot in the robot's own blind spot that never clears),
            # exclude it too, otherwise we'd pick it again forever.
            failed_frontiers.add(frontier_key((target_x, target_y)))
        elif result == TaskResult.CANCELED and mission_done():
            print('Aborting current approach: mission done.')
        elif result in (TaskResult.CANCELED, TaskResult.FAILED):
            print('Could not reach that frontier, trying another one.')
            failed_frontiers.add(frontier_key((target_x, target_y)))

    found = apriltags.found_ids
    print(f'AprilTags found: {found} ({len(found)}/{TARGET_TAG_COUNT})')

    print('Heading back to the starting point...')
    result = navigate_to(navigator, home_pose)
    if result == TaskResult.SUCCEEDED:
        print('Back at the starting point.')
    else:
        print('Could not fully return to the starting point.')

    sensors.stop()
    frontiers.stop()
    apriltags.stop()
    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
