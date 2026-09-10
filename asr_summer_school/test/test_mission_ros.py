"""Isolated ROS tests. Only localhost fake action servers; no robot commands."""
import json
import threading
import time

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
import pytest
import rclpy

from asr_summer_school.example_nav_to_pose import BoundedNavigator, Mission
from asr_summer_school.sensor_monitor import MissionSensors
from asr_summer_school.tag_mapper import TagMapper


@pytest.fixture
def ros():
    rclpy.init(args=[])
    yield
    rclpy.try_shutdown()


def test_start_and_return_services(ros):
    node = MissionSensors().start()
    client_node = Node('test_operator')
    start = client_node.create_client(Trigger, '/mission/start')
    home = client_node.create_client(Trigger, '/mission/return_home')
    try:
        assert start.wait_for_service(timeout_sec=3.)
        future = start.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=3.)
        assert not future.result().success
        node.ready = True
        future = start.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=3.)
        assert future.result().success and node.start_requested.is_set()
        assert abs(time.monotonic()-node.start_time) < 1.
        future = start.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=3.)
        assert not future.result().success
        assert home.wait_for_service(timeout_sec=3.)
        future = home.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=3.)
        assert future.result().success and node.return_requested.is_set()
    finally:
        node.stop()
        client_node.destroy_node()


def test_tag_delayed_transform_dedup_and_export(ros, tmp_path):
    node = TagMapper()
    node.output = tmp_path
    try:
        node.start(None, Trigger.Response())
        stamp = node.get_clock().now().to_msg()
        detection = AprilTagDetectionArray()
        detection.header.stamp = stamp
        detection.header.frame_id = 'camera_optical_frame'
        tag = AprilTagDetection()
        tag.family, tag.id, tag.hamming = 'tag36h11', 3, 0
        detection.detections = [tag]
        node.detect(detection)
        node.detect(detection)
        assert len(node.pending) == 1
        node.process()
        assert len(node.pending) == 1 and not node.records
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'tag36h11:3'
        tf.header.stamp = stamp
        tf.transform.translation.x = 2.
        tf.transform.rotation.w = 1.
        node.buffer.set_transform(tf, 'test')
        node.process()
        assert len(node.pending) == 0
        assert node.document()['tags'][0]['position']['x'] == 2.
        assert node.save()
        assert (tmp_path/'semantic_map.yaml').exists()
        assert json.loads((tmp_path/'observations.jsonl').read_text())['id'] == 3
        node.stop(None, Trigger.Response())
        detection.header.stamp = Time(nanoseconds=Time.from_msg(stamp).nanoseconds+1).to_msg()
        node.detect(detection)
        assert not node.pending
    finally:
        node.destroy_node()


def test_expired_tf_is_dropped(ros):
    node = TagMapper()
    try:
        node.pending.append((time.monotonic()-2., 100, 'tag36h11', 42))
        node.process()
        assert not node.pending and node.dropped == 1
    finally:
        node.destroy_node()


def test_real_action_cancel_before_next_goal(ros):
    server_node = Node('fake_navigation')
    started = threading.Event()

    def execute(handle):
        started.set()
        until = time.monotonic()+5.
        while time.monotonic() < until:
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(.02)
        handle.succeed()
        return NavigateToPose.Result()

    server = ActionServer(server_node, NavigateToPose, 'navigate_to_pose', execute,
                          cancel_callback=lambda handle: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    navigator = BoundedNavigator()
    sensors = MissionSensors()
    mission = Mission(navigator, sensors)
    try:
        assert navigator.nav_to_pose_client.wait_for_server(timeout_sec=3.)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.orientation.w = 1.
        assert navigator.goToPose(goal)
        assert started.wait(1.)
        mission.active = True
        mission.cancel()
        assert not mission.active
        assert navigator.isTaskComplete()
    finally:
        navigator.destroy_node()
        sensors.stop()
        executor.shutdown()
        thread.join(timeout=3.)
        server.destroy()
        server_node.destroy_node()


def test_speed_profiles_and_challenge_gate(ros):
    from rclpy.parameter import Parameter
    from nav2_msgs.msg import SpeedLimit
    sensors = MissionSensors().start()
    receiver = Node('speed_profile_test')
    messages = []
    receiver.create_subscription(SpeedLimit, '/mission/speed_limit', messages.append, 10)
    navigator = BoundedNavigator()
    mission = None
    try:
        sensors.set_parameters([Parameter('challenge_mode', value=True)])
        sensors.ready = True
        assert not sensors._start_mission(None, Trigger.Response()).success
        sensors.set_parameters([Parameter('speed_profiles_enabled', value=True),
                                Parameter('exploration_speed_mps', value=.12),
                                Parameter('return_limit_mps', value=.20)])
        mission = Mission(navigator, sensors)
        for state, expected in [('EXPLORE', .12), ('RETURN', .20), ('SAVE', .12)]:
            mission.state = state
            messages.clear()
            until = time.monotonic()+3.
            while time.monotonic() < until and not any(m.speed_limit == expected for m in messages):
                mission.speed_profile()
                rclpy.spin_once(receiver, timeout_sec=.05)
            assert any(m.speed_limit == expected and not m.percentage for m in messages)
    finally:
        if mission:
            mission.worker.shutdown()
        sensors.stop()
        navigator.destroy_node()
        receiver.destroy_node()
