# Onboard OAK-D mission

`example_nav_to_pose.py` is now the mission controller. It keeps the example's
BasicNavigator and background SensorMonitor approach, replacing demonstration
coordinates/timeouts with exploration, observation, return, and export states.
`tag_mapper.py` records AprilTag observations independently of navigation.

## Build on the robot

Run from the workspace root, never from a launch directory. Use this repository
and its submodule changes together. All processing and output files stay onboard.

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to asr_summer_school
source install/setup.bash
```

The existing hardware camera integration targets the installed Humble
`depthai_ros_driver` v2 API (`camera.launch.py`), not the newer `_v3` package.

## Prepare every terminal (onboard or SSH into the robot)

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=30
```

Use the same settings in every robot terminal. Localhost Fast DDS avoids relying
on a remotely configured Zenoh router. Do not run the old hardware bringup,
Gazebo, or another detector alongside `mission.launch.py`. Joystick teleop is not
started by the autonomous launch.

## Launch and start

Stopwatch-only operation (no automatic deadline):

```bash
ros2 launch asr_summer_school mission.launch.py
```

Automatic return planning with a ten-minute duration:

```bash
ros2 launch asr_summer_school mission.launch.py mission_duration_sec:=600
```

Zero or omitted duration means untimed; positive duration is seconds; negative
or non-finite durations are rejected. The countdown starts only when the start
service accepts the request, not when the process launches. Start the stopwatch
at the accepted start command. This is manual synchronization, not an electronic
connection to the stopwatch.

In another prepared terminal:

```bash
ros2 topic echo /mission/status
```

Wait for `ready: true` in the JSON string, then Ctrl+C to stop the topic display:

```bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

Manual return works in either mode:

```bash
ros2 service call /mission/return_home std_srvs/srv/Trigger "{}"
```

In untimed mode, the operator must request return early enough for travel and
saving. The robot cannot anticipate a deadline it has not been given. Return is
also triggered if exploration finishes or a verified home path is lost.
The mission accepts one start per launch; restart for a new mission/output folder.

## Physical calibration and readiness

The launch defaults to family 36h11, detectable tag edge 0.16 m, OAK-D at
(-0.062, 0, 0.245) relative to base_footprint, and 640x480 at 15 FPS.
These camera mounting and tag dimensions come from the repository, not a
measurement of your robot. Supply your actual values, for example:

```bash
ros2 launch asr_summer_school mission.launch.py \
  tag_size_m:=0.16 cam_pos_x:=-0.062 cam_pos_y:=0.0 cam_pos_z:=0.245 \
  cam_roll:=0.0 cam_pitch:=0.0 cam_yaw:=0.0
```

The OAK-D driver rectifies color images. The detector defaults to
`/camera/camera/color/image_rect` with `/camera/camera/color/camera_info`.
Override `image_topic`, `camera_info_topic`, `camera_model` (e.g. OAK-D-PRO), and
`camera_profile` if needed. Readiness checks require fresh LiDAR, odometry,
camera info, and detector messages; valid intrinsics, map and TF; active SLAM and
Nav2 lifecycle nodes; navigation/planning/rotation actions; and export services.
They do not certify physical calibration or obstacle clearance.

```bash
ros2 topic hz /camera/camera/color/image_rect
ros2 topic echo /camera/detections
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

Use the actual optical frame from camera_info if it differs. Check a detected
tag's map position with `ros2 run tf2_ros tf2_echo map tag36h11:3`, substituting its
ID. Initially test in a clear area with an accessible robot stop; sending the
return service is a navigation request, not an emergency stop. Do not publish
manual velocity commands concurrently with autonomous navigation.

## Mission configuration and results

Edit `config/mission.yaml` or supply `mission_config:=/absolute/path/config.yaml`.
Launch arguments override duration, output directory, and camera-info topic.
The file controls return reserve/speed, footprint clearance, goal/return timeout,
failed-goal cooldown, observation allowance, sensor freshness, and export period.
Keep Nav2's robot footprint consistent with the measured loaded robot too.

Timed missions reserve planned home travel at 0.10 m/s plus 30 seconds by default.
Those are initial conservative settings to tune with real measurements, not a
promise to meet a deadline under blocked routes or sensor failures. Startup and
service delays also make physical stopwatch synchronization approximate.

Results are stored under `~/challenge_results/<timestamp>/` (override
`output_root` if desired):

- `home.yaml`: recorded starting pose in map.
- `map.yaml` and `map.pgm`: reloadable occupancy map, checkpointed periodically.
- `semantic_map.yaml`: unique family/ID, map XYZ, consistency/spread and timestamps.
- `observations.jsonl`: all successfully transformed observations for later analysis.
- `mission_result.yaml`: completion state, return status, elapsed time and save outcomes.

Semantic estimates use a rolling window of at most 300 observations per tag.
Three observations within the configurable 0.30 m consistency threshold confirm
a tag; single observations remain provisional. Raw observations are retained on
disk. Delayed TF is retried for one second; expired/missing transforms are counted.
The exported median is not a joint tag/SLAM optimization: loop closure may move
map estimates, so validate accuracy and use repeated observations when evaluating
results. First/last timestamps in each tag summary refer to its retained window.

The controller checkpoints maps asynchronously during motion, confirms cancellation
before changing action type, and uses bounded retries for return. `FAILED` must
not be interpreted as a successful return, even when maps were saved. A paused or
failed sensor stream cancels motion and exports what is available.

## Validation

```bash
colcon test --packages-select asr_summer_school --ctest-args -R 'mission_.*tests|mission_tests'
colcon test-result --verbose
```

Mission tests include timing modes, frontier clearance, sparse paths crossing
obstacles, confirmation/outliers, export failures, real localhost services,
delayed TF, and cancellation using a fake navigation server. No test drives a
physical robot. ROS tests use localhost domain 91, separate from the mission.

Physical acceptance sequence: stationary detection -> manual map/TF verification
-> one autonomous frontier and manual return -> multiple frontiers -> a short
timed mission. Measure position error, CPU load, sensing delays and home travel
before a full challenge. Verify both map exports load and the robot actually
returns to its recorded starting position.

## Timed return reliability update

For a challenge run, require a configured deadline:

```bash
ros2 launch asr_summer_school mission.launch.py challenge_mode:=true mission_duration_sec:=600
```

Challenge mode rejects start without a positive duration. Ordinary launch remains
compatible with stopwatch-only operation. The deadline covers arrival, not saving.
Arrival requires fresh map TF within 0.25 m, no active mission action, and 0.5 s
of odometry below 0.02 m/s linear and 0.05 rad/s angular speed. Late arrival is
recorded as FAILED even if return and exports eventually succeed. Healthy return
attempts may continue for up to 120 seconds after the deadline; three attempts
share this bound. Untimed return shares a 120-second bound from return entry.

The initial home budget includes route distance at 0.10 m/s, sampled route turns
at 0.4 rad/s, 10 seconds for planning/cancellation, 20 seconds for recovery and the
existing 30-second safety reserve. These defaults need measurement. Exports are
outside the arrival budget. Home plans refresh asynchronously every five seconds;
missing plans or plans older than ten seconds end exploration. Odometry distance
traveled since the cached plan increases the budget. Selection is limited to six
approaches and five seconds, with a final excursion-budget check before departure.
RETURN is irreversible, and tag recording continues during travel home.

Optional faster return is disabled by default. In mission.yaml set
`speed_profiles_enabled: true` only after measuring appropriate
`exploration_speed_mps` and `return_limit_mps`. Both must be positive, exploration
must not exceed return, and return must not exceed the existing 0.22 m/s controller
maximum. No example pair is endorsed as physically calibrated. The mission sends
absolute SpeedLimit messages on /mission/speed_limit before motion and at 1 Hz,
using the return cap throughout RETURN and restoring exploration afterward.
Zero is not a stop: Nav2 interprets it as removing the restriction. Keep this topic
owned by the mission. Speed limits do not command a speed or override collision
avoidance, and faster return never reduces the conservative timing estimate.

Status/results include budget components, plan age, requested speed profile,
arrival and export timestamps, deadline compliance, odometry distance, completed
action durations, and cleanup errors. Cancellation and exports are attempted
independently. `returned_home` and `returned_before_deadline` are separate facts;
the latter is null in untimed operation.

Before enabling faster return, validate commanded and measured speed in Nav2
simulation, then measure physical stopping distances and complete return journeys
at both caps. Include winding and blocked routes, delayed planners, and repeated
short timed trials. Reload occupancy and semantic maps. Automated localhost tests
do not establish physical speed safety or full-maze deadline performance.
