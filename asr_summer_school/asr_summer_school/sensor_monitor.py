"""
Sample sensor node.

Subscribes to the robot sensor topics and keeps only the *latest* message of
each of them in its attributes, so that the main script can read them at any
time without dealing with callbacks.

The node is meant to be spun in its own thread, see `start()` / `stop()` or the
context-manager usage:

    with SensorMonitor() as sensors:
        scan = sensors.scan          # latest sensor_msgs/LaserScan (or None)
        d = sensors.min_range        # closest obstacle distance, meters
"""

import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SensorMonitor(Node):
    """Caches the last message received on every subscribed sensor topic."""

    def __init__(self, node_name='sensor_monitor', scan_topic='/scan', odom_topic='/odom'):
        super().__init__(node_name)

        # Guards every cached attribute: callbacks run in the spin thread while
        # the main thread reads the properties.
        self._lock = threading.Lock()

        self._scan = None
        self._odom = None

        self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data)

        self._executor = None
        self._thread = None

    # ------------------------------------------------------------------ #
    # Callbacks: keep them short, they only store the incoming message.
    # ------------------------------------------------------------------ #

    def _scan_callback(self, msg):
        with self._lock:
            self._scan = msg

    def _odom_callback(self, msg):
        with self._lock:
            self._odom = msg

    # ------------------------------------------------------------------ #
    # Properties: what the main script reads.
    # ------------------------------------------------------------------ #

    @property
    def scan(self):
        """Last sensor_msgs/LaserScan received, or None if nothing arrived yet."""
        with self._lock:
            return self._scan

    @property
    def odom(self):
        """Last nav_msgs/Odometry received, or None if nothing arrived yet."""
        with self._lock:
            return self._odom


    def wait_for_data(self, timeout=10.0):
        """Block until both a scan and an odometry message have been cached.

        Returns True on success, False if `timeout` seconds elapsed first.
        """
        deadline = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.scan is None or self.odom is None:
            if self.get_clock().now().nanoseconds > deadline:
                return False
            threading.Event().wait(0.05)
        return True

    # ------------------------------------------------------------------ #
    # Independent spinning.
    # ------------------------------------------------------------------ #

    def start(self):
        """Spin this node in a background thread."""
        if self._thread is not None:
            return self
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        """Stop the background thread and destroy the node."""
        if self._executor is not None and self._thread is not None:
            self._executor.shutdown()
            self._thread.join(timeout=2.0)
            self._executor.remove_node(self)
            self._executor = None
            self._thread = None
        self.destroy_node()

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
        return False

class MissionSensors(SensorMonitor):
    """Mission I/O on the existing background executor; never commands motion."""

    def __init__(self):
        import time
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import CameraInfo
        from visualization_msgs.msg import Marker
        from apriltag_msgs.msg import AprilTagDetectionArray
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from nav2_msgs.action import ComputePathToPose
        from nav2_msgs.srv import SaveMap
        from lifecycle_msgs.srv import GetState
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from tf2_ros import Buffer, TransformListener

        self.receipts = {}
        super().__init__(node_name='mission_io')
        defaults = {
            'mission_duration_sec': 0.0, 'return_reserve_sec': 30.0,
            'challenge_mode': False, 'turn_speed_rps': 0.4,
            'planning_allowance_sec': 10.0, 'recovery_allowance_sec': 20.0,
            'late_return_timeout_sec': 120.0, 'selection_timeout_sec': 5.0,
            'speed_profiles_enabled': False, 'exploration_speed_mps': 0.0,
            'return_limit_mps': 0.0,
            'return_speed_mps': 0.10, 'goal_timeout_sec': 120.0,
            'return_timeout_sec': 180.0, 'failed_cooldown_sec': 60.0,
            'save_interval_sec': 30.0, 'footprint_radius_m': 0.15,
            'output_dir': '~/challenge_results/current', 'map_frame': 'map',
            'base_frame': 'base_link', 'sensor_timeout_sec': 3.0,
            'camera_info_topic': '/camera/camera/color/camera_info',
            'detections_topic': '/camera/detections', 'observe_sec': 15.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.ready = False
        self.started = False
        self.start_requested = threading.Event()
        self.return_requested = threading.Event()
        self.grid = None
        self.frontiers = None
        self.frontier_revision = 0
        self.camera_info = None
        self.tags = {'unique': 0, 'confirmed': 0}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def cache(name, msg):
            with self._lock:
                self.receipts[name] = time.monotonic()
                if name == 'map':
                    self.grid = msg
                elif name == 'frontiers':
                    self.frontiers = msg
                    self.frontier_revision += 1
                elif name == 'camera':
                    self.camera_info = msg

        self.create_subscription(OccupancyGrid, '/map', lambda m: cache('map', m), latched)
        self.create_subscription(Marker, '/frontier_centroids',
                                 lambda m: cache('frontiers', m), latched)
        self.create_subscription(CameraInfo, self.param('camera_info_topic'),
                                 lambda m: cache('camera', m), qos_profile_sensor_data)
        self.create_subscription(AprilTagDetectionArray, self.param('detections_topic'),
                                 lambda m: cache('detections', m), 10)
        self.create_subscription(String, '/mission/tags', self._tags, 10)
        self.create_service(Trigger, '/mission/start', self._start_mission)
        self.create_service(Trigger, '/mission/return_home', self._return_home)
        from nav2_msgs.msg import SpeedLimit
        self.speed_pub = self.create_publisher(SpeedLimit, '/mission/speed_limit', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.save_map = self.create_client(SaveMap, '/map_saver/save_map')
        self.save_tags = self.create_client(Trigger, '/tag_mapper/save')
        self.start_tags = self.create_client(Trigger, '/tag_mapper/start')
        self.stop_tags = self.create_client(Trigger, '/tag_mapper/stop')
        self.lifecycle = {name: self.create_client(GetState, f'/{name}/get_state')
                          for name in ('controller_server', 'planner_server', 'bt_navigator', 'slam_toolbox')}
        self.lifecycle_pending = {}
        self.lifecycle_active = {}
        self.lifecycle_poll = 0.0

    def param(self, name):
        return self.get_parameter(name).value

    def _scan_callback(self, msg):
        import time
        super()._scan_callback(msg)
        with self._lock:
            self.receipts['scan'] = time.monotonic()

    def _odom_callback(self, msg):
        import time
        super()._odom_callback(msg)
        with self._lock:
            self.receipts['odom'] = time.monotonic()

    def _tags(self, msg):
        import json
        with self._lock:
            self.tags = json.loads(msg.data)

    def _start_mission(self, request, response):
        with self._lock:
            response.success = (self.ready and not self.started and
                                (not self.param('challenge_mode') or self.param('mission_duration_sec') > 0))
            if response.success:
                import time
                self.started = True
                self.start_time = time.monotonic()
                self.start_requested.set()
        response.message = 'Mission started' if response.success else 'Not ready or already started'
        return response

    def _return_home(self, request, response):
        response.success = self.started
        if response.success:
            self.return_requested.set()
        response.message = 'Return requested' if response.success else 'Mission has not started'
        return response

    def snapshot(self):
        with self._lock:
            return (self.grid, self.frontiers, self.frontier_revision,
                    dict(self.receipts), self.camera_info, dict(self.tags))

    def nodes_active(self):
        import time
        from lifecycle_msgs.srv import GetState
        now = time.monotonic()
        for name, future in list(self.lifecycle_pending.items()):
            if future.done():
                result = future.result()
                self.lifecycle_active[name] = bool(result and result.current_state.id == 3)
                del self.lifecycle_pending[name]
        if now-self.lifecycle_poll >= 1.0:
            self.lifecycle_poll = now
            for name, client in self.lifecycle.items():
                if not client.service_is_ready():
                    self.lifecycle_active[name] = False
                elif name not in self.lifecycle_pending:
                    self.lifecycle_pending[name] = client.call_async(GetState.Request())
        return all(self.lifecycle_active.get(name, False) for name in self.lifecycle)
