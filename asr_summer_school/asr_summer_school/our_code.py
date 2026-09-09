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
Explore the whole map by driving to frontier centroids (published by
frontier_detection_node) until none are left, while mapping every AprilTag
seen along the way onto the global map.
"""

# How long (s) the frontier list has to stay empty before we call exploration done.
# Must cover at least one slam_toolbox map_update_interval (5s, see param_slam_toolbox.yaml)
# so we don't quit right between two map updates.
NO_FRONTIER_TIMEOUT = 15.0


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


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Background nodes: they spin on their own, the main loop just reads their properties.
    sensors = SensorMonitor().start()
    frontiers = FrontierExplorer().start()
    apriltags = AprilTagMapper().start()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer='controller_server')

    # Frontiers that navigation failed to reach: skip them on future picks.
    failed_frontiers = set()

    def frontier_key(point):
        return (round(point[0], 1), round(point[1], 1))

    last_frontier_time = time.time()

    while rclpy.ok():
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
        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            # AprilTag detection and marker publishing keep running in the background.
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Reached frontier.')
        elif result in (TaskResult.CANCELED, TaskResult.FAILED):
            print('Could not reach that frontier, trying another one.')
            failed_frontiers.add(frontier_key((target_x, target_y)))

    print(f'AprilTags found: {apriltags.found_ids}')

    sensors.stop()
    frontiers.stop()
    apriltags.stop()
    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
