#! /usr/bin/env python3

import os
import subprocess
import sys
import threading
import time

from apriltag_msgs.msg import AprilTagDetectionArray
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool
import tf2_py as tf2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

"""
Explore the maze with explore_lite (github.com/robo-friends/m-explore-ros2), mapping
every AprilTag seen along the way onto the global map, until all tags are found or the
mission time budget runs out -- then return to the starting point.
"""

# Total time budget (s) for finding AprilTags before heading back to the start.
MISSION_TIME_LIMIT = 4 * 60.0

# Number of distinct AprilTags placed in the maze.
TARGET_TAG_COUNT = 12

# Robot frame used to look up poses in the map. Matches `robot_base_frame` in
# param_nav2.yaml and explore_lite_params.yaml.
ROBOT_FRAME = 'base_link'

# explore_lite's own params file, next to this script's package (not the installed
# share dir: our_code.py is always run straight from source, not via ros2 run).
EXPLORE_PARAMS_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'explore_lite_params.yaml')


def use_sim_time_from_args():
    """Read a `use_sim_time:=true|false` arg, matching this repo's launch-file convention.

    Every node here needs to agree on the same clock: a node using wall-clock time
    against a TF tree stamped with Gazebo's simulated clock (or vice versa) makes every
    lookup fail extrapolation, since the requested time never matches available data.
    """
    for arg in sys.argv[1:]:
        if arg == 'use_sim_time:=true':
            return True
        if arg == 'use_sim_time:=false':
            return False
    return False


class ExploreStatusMonitor(Node):
    """Caches the latest status published by explore_lite on `/explore/status`."""

    def __init__(self, node_name='explore_status_monitor', status_topic='/explore/status',
                 use_sim_time=False):
        super().__init__(node_name, parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])

        self._lock = threading.Lock()
        self._status = None

        self.create_subscription(
            ExploreStatus, status_topic, self._status_callback,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._executor = None
        self._thread = None

    def _status_callback(self, msg):
        with self._lock:
            self._status = msg.status

    @property
    def complete(self):
        """True once explore_lite reports it has run out of frontiers."""
        with self._lock:
            return self._status == ExploreStatus.EXPLORATION_COMPLETE

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
                 marker_topic='/apriltag_markers', map_frame='map', use_sim_time=False):
        super().__init__(node_name, parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])

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

            stamp = self.get_clock().now().to_msg()
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            square = Marker()
            square.header.frame_id = self.map_frame
            square.header.stamp = stamp
            square.ns = 'apriltags'
            square.id = tag.id
            square.type = Marker.CUBE
            square.action = Marker.ADD
            square.pose.position.x = x
            square.pose.position.y = y
            square.pose.position.z = z
            square.pose.orientation.w = 1.0
            square.scale.x = square.scale.y = 0.25
            square.scale.z = 0.05
            square.color.a = 1.0

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = stamp
            label.ns = 'apriltag_labels'
            label.id = tag.id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.text = str(tag.id)
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.2
            label.color.a = 1.0
            label.color.r = label.color.g = label.color.b = 1.0

            with self._lock:
                new_tag_found = new_tag_found or tag.id not in self._found
                self._found[tag.id] = (square, label)

        if new_tag_found:
            self._publish_markers()
            self.get_logger().info(f'AprilTags found so far: {self.found_ids}')

    def _publish_markers(self):
        with self._lock:
            markers = [marker for pair in self._found.values() for marker in pair]
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


def wait_for_start_pose(navigator, buffer, map_frame, robot_frame, timeout=15.0):
    """Block until map_frame -> robot_frame is available and return PoseStamped."""
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
    """Drive to goal_pose, calling should_abort() to cancel early if conditions met."""
    if not navigator.goToPose(goal_pose):
        return TaskResult.FAILED

    i = 0
    while not navigator.isTaskComplete():
        if should_abort is not None and should_abort():
            navigator.cancelTask()
            break
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f'Estimated time of arrival: {eta:.0f} seconds.')

    return navigator.getResult()


def main():
    rclpy.init()

    use_sim_time = use_sim_time_from_args()
    print(f'use_sim_time={use_sim_time}')

    navigator = BasicNavigator()
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])

    # Background nodes: they spin on their own, the main loop just reads their properties.
    apriltags = AprilTagMapper(use_sim_time=use_sim_time).start()
    explore_status = ExploreStatusMonitor(use_sim_time=use_sim_time).start()

    navigator.waitUntilNav2Active(localizer='controller_server')

    print('Looking up starting pose...')
    home_pose = wait_for_start_pose(navigator, apriltags.buffer, apriltags.map_frame, ROBOT_FRAME)
    print(f'Start pose: x={home_pose.pose.position.x:.2f}, y={home_pose.pose.position.y:.2f}')

    mission_deadline = time.time() + MISSION_TIME_LIMIT

    # explore_lite (github.com/robo-friends/m-explore-ros2) does the actual frontier
    # exploration: picking, blacklisting unreachable ones, and driving via Nav2 itself.
    # We just supervise it against our own stopping conditions and pause it (rather than
    # kill it outright) so it cancels its in-flight goal cleanly.
    resume_pub = navigator.create_publisher(Bool, 'explore/resume', 10)
    print('Starting explore_lite...')
    explore_proc = subprocess.Popen([
        'ros2', 'run', 'explore_lite', 'explore',
        '--ros-args', '--params-file', EXPLORE_PARAMS_FILE,
        '-p', f'use_sim_time:={str(use_sim_time).lower()}',
    ])

    try:
        while rclpy.ok():
            if len(apriltags.found_ids) >= TARGET_TAG_COUNT:
                print(f'Found all {TARGET_TAG_COUNT} AprilTags!')
                break
            if time.time() >= mission_deadline:
                print(f'Mission time budget ({MISSION_TIME_LIMIT / 60:.0f} min) reached.')
                break
            if explore_status.complete:
                print('explore_lite ran out of frontiers: exploration complete.')
                break
            time.sleep(0.5)

        print('Stopping exploration...')
        resume_pub.publish(Bool(data=False))
    finally:
        explore_proc.terminate()
        try:
            explore_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            explore_proc.kill()
            explore_proc.wait()

    found = apriltags.found_ids
    print(f'AprilTags found: {found} ({len(found)}/{TARGET_TAG_COUNT})')

    print('Heading back to the starting point...')
    result = navigate_to(navigator, home_pose)
    if result == TaskResult.SUCCEEDED:
        print('Back at the starting point.')
    else:
        print('Could not fully return to the starting point.')

    apriltags.stop()
    explore_status.stop()
    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
