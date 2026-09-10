# TurtleBot challenge implementation notes

Updated: 2026-09-10. Current priority: **return home before the deadline**, then
maximize useful tag detections within that constraint.

This document records the changes made to implement the README challenge using
the existing navigation example. The target is a **physical TurtleBot3 with an
OAK-D camera, running all processing onboard with ROS 2 Humble**.

The implementation has been built and software-tested in this workspace. It
has **not been deployed to or validated on the physical robot**. Camera mounting,
tag dimensions, obstacle clearance, and full mission performance still need
physical verification.

For the detailed operating guide, see [MISSION.md](asr_summer_school/MISSION.md).

## 1. How the seven challenge requirements are covered

| Requirement | Implementation |
|---|---|
| Build a 2D occupancy map | SLAM Toolbox processes filtered LiDAR scans and publishes `/map`. |
| Explore autonomously | Frontier detection supplies candidate locations; the controller selects reachable approach poses and sends Nav2 goals. |
| Detect as many AprilTags as possible | Detection runs during travel, observation turns, and return. The robot explores until return is requested, required by its budget, or exploration finishes. This is a heuristic, not a guarantee of finding the maximum possible number. |
| Associate detections with unique IDs | The tag mapper groups observations by tag family and numeric ID. |
| Estimate/store tag positions in `map` | Timestamped TF lookups transform detections into map coordinates; repeated observations produce a robust position estimate. |
| Return to the start | The start pose is recorded through TF; the controller navigates back automatically in timed mode or on an operator request. |
| Save occupancy and semantic maps | Periodic checkpoints and final exports write both maps and mission results onboard. |

## 2. Files changed or added

Paths below are relative to this repository directory.

| File | What it now does |
|---|---|
| [example_nav_to_pose.py](asr_summer_school/asr_summer_school/example_nav_to_pose.py) | The original example is now the mission controller. Keeps the BasicNavigator pattern, adds bounded goal submission, exploration/return logic, optional timing, status, and export coordination. |
| [sensor_monitor.py](asr_summer_school/asr_summer_school/sensor_monitor.py) | Keeps the original sensor helper and adds `MissionSensors`: background subscriptions, TF, lifecycle checks, planning/export clients, and operator services. |
| [mission_logic.py](asr_summer_school/asr_summer_school/mission_logic.py) | ROS-independent timing, path length, known-space footprint checks, tag statistics, and atomic YAML writing. |
| [tag_mapper.py](asr_summer_school/asr_summer_school/tag_mapper.py) | Processes detections independently of navigation, retries delayed TF, groups observations, and saves semantic data. |
| [mission.launch.py](asr_summer_school/launch/mission.launch.py) | Starts the complete physical robot mission stack, with OAK-D and simulation time disabled. |
| [mission.yaml](asr_summer_school/config/mission.yaml) | Mission thresholds, timeouts, return reserve, sensor topics, and export interval. |
| [oakd_mission.yaml](asr_summer_school/config/oakd_mission.yaml) | RGB-only OAK-D configuration without neural-network or IMU processing. |
| [param_nav2.yaml](asr_summer_school/config/param_nav2.yaml) | Updates the older recovery-server configuration to Humble's behavior server, disables duplicate voxel processing of LiDAR, uses a 0.15 m radius, and prevents planning through unknown space. Simulation launch arguments can still override time settings. |
| [frontier_detection_node.cpp](asr_summer_school/src/frontier_detection_node.cpp) | Uses TF for the robot position instead of assuming a `/pose` publisher exists. The old pose-topic mode remains available via `use_tf_pose:=false`. |
| [CMakeLists.txt](asr_summer_school/CMakeLists.txt) and [package.xml](asr_summer_school/package.xml) | Install the added executable, declare dependencies, and register mission tests. |
| [test_mission.py](asr_summer_school/test/test_mission.py) | Tests mission decisions and controller behavior without a robot. |
| [test_mission_ros.py](asr_summer_school/test/test_mission_ros.py) | Tests localhost services, delayed transforms, and action cancellation with a fake navigation server. |

The existing frontier visualization test was adjusted to use pose-topic mode
because it intentionally has no robot TF. The repository README now links to the
new mission guide. Existing simulation AprilTag edits were preserved.

## 3. What changed in the example code

The original example sent a fixed destination `(2.0, -0.5)`, printed odometry and
feedback, changed destination after 18 seconds, and demonstrated cancellation
after 600 seconds. Those example destinations and timeouts have been replaced.

The mission flow is now:

```text
READY: wait for sensors, TF, map, navigation, and start command
  |
  v
Record home and begin recording tags
  |
  v
EXPLORE: choose a reachable frontier and navigate
  |
  v
OBSERVE: turn to expose the camera to the surrounding area
  |
  +---- repeat exploration while permitted
  |
  v
RETURN: cancel current action, then navigate home
  |
  v
SAVE: stop tag recording and export maps/results
  |
  v
FINISHED or FAILED
```

Tag processing runs independently throughout exploration and return. A sensor or
TF failure can abort motion and lead to failure/export handling rather than
continuing the normal sequence.

The controller uses map-frame TF for home/current positions. Raw odometry is
normally in `odom` and must not be treated as a `map` coordinate directly.
Planning requests use a separate action client so they do not overwrite the
navigation task tracked by BasicNavigator.

## 4. Duration versus physical stopwatch

| Setting | Behavior |
|---|---|
| Duration omitted or `mission_duration_sec:=0` | No automatic deadline. The operator watches the stopwatch and requests return early enough. |
| Positive duration, such as `mission_duration_sec:=600` | A wall-clock countdown starts when the start command is accepted. The robot budgets for arrival before the deadline; final saving may finish afterward. |
| Negative or non-finite duration | Rejected. |

Start the physical stopwatch when the start service accepts the request. This is
manual synchronization; the robot is not connected to the stopwatch. Duration is
chosen before the run, not dynamically changed during a mission.

In timed mode, the budget includes planned distance at 0.10 m/s, sampled turns
at 0.4 rad/s, 10 seconds for planning/cancellation, 20 seconds for recovery, and
a 30-second safety margin. Section 10 explains the calculation and its limits.
An unspecified stopwatch deadline cannot trigger an automatic time-based return.
Blocked routes or failures can prevent return even with a configured duration.

## 5. Commands to run on the robot

Build from the workspace root after transferring the complete updated source:

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to asr_summer_school
source install/setup.bash
```

Prepare **every terminal**, including SSH terminals connected to the robot:

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=30
```

These settings keep ROS communication onboard without requiring a remote Zenoh
router. Run operator commands through a terminal on the robot or SSH into it.

Terminal 1 — choose **one** launch command:

```bash
# Stopwatch-only operation
ros2 launch asr_summer_school mission.launch.py

# Recommended challenge operation: require a positive duration (10 minutes here)
ros2 launch asr_summer_school mission.launch.py \
  challenge_mode:=true mission_duration_sec:=600
```

Do not also launch the old bringup, Gazebo, another detector, or competing teleop.

Terminal 2 — inspect readiness:

```bash
ros2 topic echo /mission/status
```

The topic carries a JSON string. Wait for `ready: true`, then stop the echo with
Ctrl+C and start:

```bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

Request return in either timing mode:

```bash
ros2 service call /mission/return_home std_srvs/srv/Trigger "{}"
```

Return is a navigation request, not an emergency stop. Use the robot's accessible
stop procedure during physical testing. One start is accepted per launch; restart
the launch for a new mission.

## 6. Camera and tag measurements

The hardware launch uses the installed Humble OAK-D v2 driver API. Its defaults
are 640×480 at 15 FPS, family `36h11`, and a detectable tag edge of 0.16 m.
The camera mounting defaults are copied from the repository, not measured on your
robot. Replace them with physical measurements:

```bash
ros2 launch asr_summer_school mission.launch.py \
  tag_size_m:=0.16 \
  cam_pos_x:=-0.062 cam_pos_y:=0.0 cam_pos_z:=0.245 \
  cam_roll:=0.0 cam_pitch:=0.0 cam_yaw:=0.0
```

Positions are relative to `base_footprint`; angles are radians. The tag edge is
the detectable square, not the entire paper or its outer white margin.
For OAK-D Pro, supply `camera_model:=OAK-D-PRO`.

The default detection inputs are:

```text
/camera/camera/color/image_rect
/camera/camera/color/camera_info
```

These differ from the Gazebo camera topics used earlier. Inspect the physical
topics and override `image_topic`/`camera_info_topic` if necessary.

```bash
ros2 topic echo /camera/camera/color/camera_info --once
ros2 topic hz /camera/camera/color/image_rect
ros2 topic echo /camera/detections
ros2 run tf2_ros tf2_echo map tag36h11:3
```

Replace `3` with a currently visible tag ID. Tag pose coordinates come from TF;
the nine `homography` values in a detection describe a 3×3 image projection, not
nine physical distances.

## 7. Saved files and interpretation

Each launch chooses a new output directory:

```text
~/challenge_results/<timestamp>/
├── home.yaml
├── map.yaml
├── map.pgm
├── semantic_map.yaml
├── observations.jsonl
└── mission_result.yaml
```

- `home.yaml`: starting pose in the map frame.
- `map.yaml` and `map.pgm`: occupancy map metadata and image.
- `semantic_map.yaml`: family/ID, estimated map XYZ, confirmation state, counts,
  spread, and observation timestamps.
- `observations.jsonl`: successfully transformed observations retained for analysis.
- `mission_result.yaml`: state, reason, return status, elapsed time, and export success.

Maps are checkpointed periodically, initially every 30 seconds, and exported at
completion. Confirm export-success fields rather than assuming file creation
means a successful mission.

Tag estimates use the latest 300 observations per tag. Three spatially consistent
observations within an initial 0.30 m threshold confirm a tag; single observations
remain provisional. The estimate uses a robust median. Repeated observations do
not increase the unique-tag count. Missing TF is retried briefly, then counted as
dropped if unavailable.

This is not joint SLAM/tag optimization. Loop closure can affect map coordinates;
physical accuracy must be measured. The retained observations contain timestamped
map-frame XYZ, family and ID; they do not retain camera-relative poses or an
optimized SLAM trajectory sufficient to reconstruct corrected positions. Summary timestamps refer to the retained observation window.

## 8. Where to look when something is unclear

| Symptom or question | First place to check |
|---|---|
| Start request rejected | `/mission/status`, fresh scan/odom/camera/detection topics, TF, SLAM/Nav2 lifecycle states. |
| No camera detections | Rectified image topic, camera calibration, visible tag family and size. Empty arrays can mean no tag is visible. |
| Detections exist but no map positions | TF chain to `map`, timestamps, and dropped-observation count in the semantic map. |
| Robot stays still during exploration | Frontier availability, known-free clearance, valid paths, and failed-goal cooldown. |
| Robot returns early | Operator request, time reserve, unavailable home route, or exhausted frontiers. |
| Wrong tag distances | Measured tag edge and camera calibration. |
| Wrong map-frame direction/offset | Physical camera mounting transform and optical-frame convention. |
| Need different return/goal behavior | `config/mission.yaml` and `example_nav_to_pose.py`. |
| Need different camera settings | Launch arguments and `config/oakd_mission.yaml`. |
| Need different robot clearance | Keep mission footprint checks and `config/param_nav2.yaml` consistent with the measured robot. |
| `FAILED` at completion | Inspect `mission_result.yaml`; saved maps alone do not establish successful return. |

## 9. What was tested, and what remains

Completed locally:

- The ROS package builds successfully.
- All 32 mission test cases passed directly after the timed-return update.
- Registered `colcon` tests passed with zero errors/failures; its summary counts
  34 entries because it also includes the two test-suite wrappers.
- Tests cover timed/untimed decisions, path clearance, outliers, exports,
  start/return services, delayed TF, duplicates, and cancellation against a fake
  navigation server. Added coverage includes return budgets and turns, stale/failed
  home plans, expired selection, late-return limits, stationary arrival, cleanup
  after cancellation failure, mocked complete return flows, challenge-mode start
  rejection, and speed messages delivered over localhost ROS.
- Launch arguments and installed Nav2 plugin libraries were checked.

Re-run the registered tests with:

```bash
colcon test --packages-select asr_summer_school --ctest-args -R mission_
colcon test-result --verbose
```

The ROS tests use localhost domain 91 and do not command physical hardware.

Still required on the robot:

1. Verify measured camera mounting, printed tag dimensions, and robot footprint.
2. Verify stationary detection and map-frame tag positions.
3. Verify mapping and one autonomous frontier followed by manual return.
4. Test multiple frontiers and both timing modes with short missions.
5. Measure CPU load, sensor delays, tag-position error, and actual return time.
6. Confirm the exported occupancy and semantic maps can be loaded and match the
   observed environment before attempting the full challenge.

## 10. Crucial behavior in the timed-return update

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

### Budget versus actual speed

The controller evaluates approximately:

```text
home budget = path length / return_speed_mps
            + sampled heading changes / turn_speed_rps
            + planning_allowance_sec
            + recovery_allowance_sec
            + return_reserve_sec
            + distance traveled since cached plan / return_speed_mps
```

`return_speed_mps` is an **estimate**, not a commanded velocity. Nav2 already
allows up to 0.22 m/s in the existing configuration. Enabling profiles restricts
exploration to one measured cap and relaxes that restriction to a second measured
cap on return; it does not raise the controller maximum. The robot may move much
slower because of turns, obstacles, controller behavior or recovery.

The route-turn calculation uses headings between points separated by roughly
0.25 m and includes initial heading alignment. It is an approximation, not a
trajectory simulation. Planning and recovery allowances are budgeting inputs;
they are not separate timers forcibly terminating every Nav2 recovery.

Candidate planning considers at most six approach poses, not six distinct
frontiers. Its five-second limit covers the planning loop; generating the list
and checking free-space footprints happen before that timer. Departure validation
has its own five-second planning budget and rechecks the full excursion cost.
The initial home plan is synchronous; subsequent refresh requests run in a
worker, serviced through the sensor executor. Foreground planning waits for an
in-flight refresh before submitting another request to the same planner.

### Return, arrival and failure are separate facts

- RETURN never resumes exploration. Camera detections continue opportunistically.
- A timed mission may continue home until deadline + `late_return_timeout_sec`
  (120 s by default). There are at most three attempts and a 180 s per-navigation
  timeout; neither retries nor a later return start reset the absolute deadline.
- Untimed operation uses return-entry time + 120 s for its shared return window.
- Arrival is recorded after the stationary verification interval, not when the
  robot first crosses the 0.25 m boundary. Verification still requires healthy
  sensors and TF. This can turn a last-second approach into a recorded late return.
- `FINISHED` requires verified arrival, on-time arrival for timed runs, and both
  final map exports succeeding. Saving after the deadline does not itself fail
  the arrival criterion. A late robot can have `returned_home: true` and still
  finish in `FAILED`.
- Failure cleanup independently attempts cancellation, tag-stop/export and final
  exports. Failed cancellation does not prove that the robot stopped. Export
  attempts cannot guarantee files if ROS is down, the process is killed, or the
  storage device fails.

### Reading the new result fields

| Field | Interpretation |
|---|---|
| `return_budget` | Components of the last accepted home-route estimate; its total excludes subsequent odometry adjustment. |
| `estimated_home_sec` (status) | Adjusted estimate including planning/recovery, but excluding the separately added safety reserve. |
| `home_plan_age_sec` | Age since the accepted plan request; may keep increasing during return because exploration refreshes have stopped. |
| `requested_speed_profile` | Requested cap selection, not acknowledgement or measurement of actual speed. |
| `home_arrival_elapsed_sec` | Elapsed monotonic time at verified stationary arrival; null if never verified. |
| `returned_before_deadline` | True/false for timed runs, null for untimed runs. |
| `export_completion_elapsed_sec` | Time after final export attempts, including failures; inspect success flags too. |
| `cleanup_errors` | Exceptions/timeouts recorded during cleanup; inspect save flags and logs as well. |
| `odometry_distance_m` | Distance accumulated from sampled odometry positions, not a precise ground-truth traveled distance. |
| `motion_samples` | Durations of actions reaching the normal completion path, with return/observation flags; canceled/timed-out actions are not comprehensively logged. |

## 11. Weak points, failure modes and follow-up work

These are remaining limitations, not features claimed to be solved by the update.

| Priority / weak point | Failure or score impact | Suggested work and acceptance check |
|---|---|---|
| High: timing estimates are uncalibrated | Slow turns, congestion, recovery or onboard load can exhaust the reserve. Faster limits do not guarantee faster travel. | Measure repeated complete routes and blocked-route trials. Tune speed estimates/allowances to conservative observed performance; log canceled actions and plan delays too. |
| High: home route becomes blocked | RETURN starts, but a reachable route may not exist; three retries can fail. | Exercise temporary and persistent blockages, assess alternate routes, and verify shared timeout behavior with real Nav2 recovery. |
| High: sensor loss during return | Current health checks require scan, odometry, camera info and detector messages even on the way home. A camera-only outage can abort an otherwise navigable return. | Design state-specific health requirements: preserve LiDAR/odometry/TF requirements, but consider allowing return after perception-only failure. Test each failed stream separately. |
| High: cancellation or late action acceptance | A server that does not acknowledge cancellation may keep moving. Late-acceptance cancellation is asynchronous and must be exercised with a delayed server. | Test delayed acceptance and refused/hung cancellation, including shutdown. Validate the robot's independent stop mechanism; do not substitute a speed-limit message for stopping. |
| High: faster-profile delivery is not confirmed | Publishing a speed cap does not prove Nav2 received/applied it. Competing publishers or controller restarts can alter the effective limit. | Verify commanded and measured speeds at both caps and after a controller restart. Add acknowledgement/diagnostics or a dedicated speed-limit owner before relying on profile enforcement. |
| High: tag coordinates after loop closure | Historical map-frame positions are not reoptimized; median estimates can disagree with the final occupancy map. | Store camera-relative observations with trajectory/keyframe associations, then recompute against optimized poses. Test against known tag positions after loop closure. |
| Medium: stale frontier publisher | Sensor health does not check frontier freshness. Empty-count termination requires new frontier revisions; an untimed mission can wait indefinitely if they stop. | Add a frontier freshness watchdog and distinguish fresh-empty results from unavailable detection. |
| Medium: repeated zero-gain frontier visits | Temporary cooldown expires even when a visit reveals no new cells or tags; only the nearest three local approaches are retained per frontier. | Track information gain and camera coverage, penalize repeated unproductive visits, and try alternate approaches. |
| Medium: planner contention and selection cost | Candidate generation is outside the five-second loop budget; a large frontier set or expensive clearance checks can delay supervision. | Bound the entire selection operation and test large maps. Stress delayed/preempted planner requests while navigation remains active. |
| Medium: LiDAR coverage is not camera coverage | A mapped room can still contain unseen tags. A successful spin does not establish successful detection. | Test the actual detector on recorded/simulated/physical images; track useful viewpoints and detection range, blur and viewing angles. |
| Medium: exports and storage | Timeouts, full disks, process death or in-flight checkpoints can leave incomplete or mismatched occupancy/semantic output. | Test service failures and disk errors; add export generations/manifests if consistent paired snapshots are required. Reload actual files, rather than checking existence only. |
| Medium: calibration and home tolerance | Wrong camera extrinsics/tag size distort coordinates; 0.25 m home tolerance may differ from the challenge's scoring rule. | Measure calibration and footprint; confirm official arrival/position tolerances and scoring, then align checks and Nav2 tolerances. |
| Medium: configuration precedence | Launch explicitly supplies duration and challenge mode, overriding those YAML values. Custom frame/topic parameters are not consistently propagated through the entire launch. | Supply duration/challenge mode as launch arguments. Keep default map/base/topics unless the complete stack is updated and tested together. |

Recommended next order: validate complete timed return in simulation, measure
physical timing and stopping behavior, improve return-specific sensor handling,
then optimize tag-search coverage and correct semantic positions after loop
closure. Keep speed escalation optional until those measurements support it.

### What the detector tests actually prove

The mapper tests construct `AprilTagDetectionArray` messages and insert known TF
transforms. This is **synthetic input**, not a second detector implementation.
It verifies downstream mapping logic, delayed TF and exports. It does not verify
that camera images produce detections or correct poses. Running `apriltag_ros`
on Gazebo camera images uses the real detector on simulated imagery and tests
more of the pipeline. Physical imagery is still needed for calibration, range,
lighting and motion-blur evaluation. Full-maze Nav2 simulation and physical
acceptance have not been completed in this update.

## 12. Additional terminal commands: configuration, inspection and results

Run the environment setup in section 5 in every terminal first. Terminal 1 owns
the complete hardware launch; no separate Nav2, SLAM, camera or detector command
is needed. Terminal 2 starts/returns the mission. Terminal 3 can keep status or
sensor monitoring running. Localhost-only ROS means remote operators must SSH
into the robot rather than launch these clients on their own laptop.

### Use a private configuration copy

```bash
mkdir -p ~/challenge_config
cp -n ~/ros_ws/src/asr_summer_school_challenge/asr_summer_school/config/mission.yaml \
  ~/challenge_config/mission.yaml
nano ~/challenge_config/mission.yaml
```

Keep the `/**: ros__parameters:` YAML nesting. For faster return, edit
`speed_profiles_enabled`, `exploration_speed_mps` and `return_limit_mps` only after
measuring caps. Do not enable profiles with their default zero values. Select a
positive challenge duration with the launch argument even when using a YAML copy:

```bash
ros2 launch asr_summer_school mission.launch.py \
  challenge_mode:=true mission_duration_sec:=600 \
  mission_config:=$HOME/challenge_config/mission.yaml \
  output_root:=$HOME/challenge_results
```

This is an alternative to the section 5 launch, not an additional launch.
Calibration arguments from section 6 can be appended to this same command.
After editing source configuration without a private copy, rebuild/source as in
section 5. Restart the mission to apply configuration; live parameter changes
are not a supported tuning workflow.

### Inspect services, sensing and requested speeds

Run these individually as needed; Ctrl+C stops a streaming monitor, not the
mission in terminal 1.

```bash
ros2 topic echo /mission/status
ros2 topic echo /mission/tags
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /camera/camera/color/image_rect
ros2 topic echo /camera/detections
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /mission/speed_limit
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

`/mission/speed_limit` only receives mission caps when profiles are enabled.
`/cmd_vel` is the commanded velocity; odometry reports estimated actual velocity.
Neither establishes physical stopping distance. Inspect `reason` in status if
return starts early. Use the start/return service commands from section 5; the
return command is not an emergency stop. Once the run has ended and the robot
is stopped, Ctrl+C in terminal 1 shuts down the remaining launch processes.

### Inspect the latest result directory

```bash
python3 - <<'PYRESULT'
from pathlib import Path
import yaml
root = Path.home() / 'challenge_results'
runs = sorted(p for p in root.iterdir() if p.is_dir()) if root.exists() else []
if not runs:
    raise SystemExit('No result directories found')
run = runs[-1]
print('Results:', run)
for name in ('mission_result.yaml', 'semantic_map.yaml', 'map.yaml'):
    path = run / name
    print('\n' + name)
    print(yaml.safe_dump(yaml.safe_load(path.read_text()), sort_keys=False)
          if path.exists() else 'Not written yet or export failed')
PYRESULT
```

Use the actual output root if overridden. YAML parsing is only a basic integrity
check; load the occupancy map in a separate validation session and compare tags
to the scene. Do not start another map server on the live mission's `/map` topic.

### Re-run the software checks

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select asr_summer_school
colcon test --packages-select asr_summer_school --ctest-args -R mission_
colcon test-result --test-result-base build/asr_summer_school --verbose
ros2 launch asr_summer_school mission.launch.py --show-args
```

The recorded result is 32 test cases / 34 colcon entries including wrappers,
zero errors/failures. `--show-args` inspects launch options without starting the
robot. These checks do not replace the full-stack simulation and physical trials
listed above.
