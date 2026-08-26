# formation

ROS 2 Humble package for PX4/Gazebo multi-UAV formation control.

This package is designed for three PX4 UAVs named `MAV1`, `MAV2`, and `MAV3`. It currently supports synchronized takeoff/landing, leader-follower formation control, leader command relay, ENU/NED coordinate conversion, CPF velocity-based collision avoidance, and mission-level safety checks.

The current main workflow is:

```text
mission_node
    synchronized takeoff / landing / mission state / safety

leader_command_node
    receives /formation/leader_input and publishes /formation/command

leader_control_node
    controls only the leader during FORMATION

follower_formation_node
    controls follower UAVs relative to the latest leader position
```

## Main features

- Three-UAV synchronized takeoff and landing
- Takeoff stabilization with Offboard/Arm retry, armed barrier, and altitude ramp
- PX4 Offboard control through `px4_msgs`
- Leader-follower formation mode
- Centralized and distributed control code kept for development/extension
- Leader movement command through `/formation/leader_input`
- Leader selection service
- Formation shape and spacing update through leader command/status
- Slot assignment relative to the latest leader position
- Formation complete detection with position tolerance, speed tolerance, and hold duration
- ROS ENU to PX4 NED coordinate conversion
- Control Potential Field, CPF, velocity-based UAV avoidance
- Safety checks for distance, speed, altitude, geofence/fence, timeout, and setpoint validity
- Rectangular ENU world fence for real-flight testing, with velocity braking near boundaries

## Tested environment

This package was developed around:

- Ubuntu with ROS 2 Humble
- PX4 Autopilot v1.17 SITL
- `px4_msgs` compatible with PX4 v1.17
- Gazebo Sim
- Micro XRCE-DDS Agent

Other versions may work, but PX4 message names and fields can change between PX4 / `px4_msgs` versions.

## Repository layout

```text
formation/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── mission.yaml
│   ├── mission_real.yaml
│   ├── leader_command.yaml
│   ├── leader_control.yaml
│   ├── follower_formation.yaml
│   ├── follower_formation_real.yaml
│   ├── distributed.yaml
│   └── formation.yaml
├── docs/
│   └── code_file_explanations/
├── formation/
│   ├── main_node.py
│   ├── mission_manager.py
│   ├── vehicle_interface.py
│   ├── leader_command_node.py
│   ├── leader_control_node.py
│   ├── follower_formation_node.py
│   ├── distributed_vehicle_node.py
│   ├── formation_controller.py
│   ├── formation_shapes.py
│   ├── cpf_avoidance.py
│   ├── safety_manager.py
│   ├── leader_manager.py
│   └── coordinate_convert.py
├── launch/
│   ├── formation.launch.py
│   ├── formation_real.launch.py
│   └── multi_PX4.launch.py
├── msg/
│   ├── FormationCommand.msg
│   ├── FormationError.msg
│   └── FormationStatus.msg
├── scripts/
│   ├── mission_node
│   ├── leader_command_node
│   ├── leader_control_node
│   ├── follower_formation_node
│   ├── formation_node
│   ├── distributed_vehicle_node
│   └── monitor_mav_dds.sh
└── srv/
    ├── SetFormation.srv
    └── SetLeader.srv
```

## Dependencies

Install ROS 2 Humble first. This package also requires `px4_msgs` in the same ROS 2 workspace.

Example workspace:

```text
uav_ws/
└── src/
    ├── formation/
    └── px4_msgs/
```

For PX4 v1.17, use a `px4_msgs` version that matches the PX4 message definitions used by your PX4-Autopilot checkout.

## Build

Clone this package into a ROS 2 workspace:

```bash
mkdir -p ~/uav_ws/src
cd ~/uav_ws/src
git clone https://github.com/ZHG169/formation.git
```

Make sure `px4_msgs` is also available in `~/uav_ws/src`, then build:

```bash
cd ~/uav_ws
colcon build --packages-select formation
source install/setup.bash
```

If custom msg/srv files were changed, rebuild and source again:

```bash
colcon build --packages-select formation
source install/setup.bash
```

## Before running

Start these first:

1. PX4 SITL / Gazebo with three UAV instances: `MAV1`, `MAV2`, `MAV3`
2. Micro XRCE-DDS Agent, usually on UDP port `8888`
3. Source the ROS 2 workspace where `formation` and `px4_msgs` were built

The expected PX4 DDS namespaces are:

```text
/MAV1/fmu/...
/MAV2/fmu/...
/MAV3/fmu/...
```

The package assumes ROS/Gazebo high-level logic uses ENU coordinates:

```text
x = east
y = north
z = up
```

PX4 Offboard setpoints are converted to NED before publishing.

## Run formation package

PX4/Gazebo startup is kept separate. After the three UAVs and Micro XRCE-DDS Agent are ready, launch the formation package:

```bash
ros2 launch formation formation.launch.py
```

By default, the launch file uses `leader_follower` mode and starts:

```text
mission_node
leader_command_node
leader_control_node
follower_formation_node
```

You can also pass the mode explicitly:

```bash
ros2 launch formation formation.launch.py control_mode:=leader_follower
```

Centralized and distributed modes are still available for development:

```bash
ros2 launch formation formation.launch.py control_mode:=centralized
ros2 launch formation formation.launch.py control_mode:=distributed
```

### Real-flight launch profile

For real-world testing with OptiTrack / external vision, use:

```bash
ros2 launch formation formation_real.launch.py
```

This launch file reuses the same node graph as `formation.launch.py`, but loads real-flight parameter files:

```text
config/mission_real.yaml
config/follower_formation_real.yaml
```

The real profile assumes PX4 `vehicle_local_position_v1` is already in a shared external-vision / OptiTrack coordinate frame. Therefore `vehicle_origins_enu` is set to zero offsets:

```yaml
vehicle_origins_enu: [
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0
]
```

The Gazebo profile keeps the original spawn offsets in `mission.yaml` and `follower_formation.yaml`.

## Mission flow

The normal leader-follower flow is:

```text
IDLE
  -> WAITING_READY
  -> OFFBOARD_WARMUP
  -> TAKEOFF
  -> WAITING_LEADER_COMMAND
  -> FORMATION
  -> LANDING / EMERGENCY_LANDING
```

During `TAKEOFF`, `mission_node` owns all UAV setpoints. The takeoff sequence is intentionally conservative:

```text
1. Warm up Offboard setpoints at the captured ground/home position
2. Send Offboard + Arm commands
3. Retry Offboard/Arm for UAVs that are still pending
4. Hold all UAVs at their captured ground/home height until all are armed and in Offboard
5. After all are ready, ramp the altitude setpoint toward takeoff_height
```

This avoids the case where one UAV arms earlier and starts climbing while the others are still waiting for PX4 to accept Offboard/Arm.

After all UAVs reach the takeoff height, the mission waits in `WAITING_LEADER_COMMAND`. Formation motion does not start until a leader movement command is received.

During `FORMATION`:

- `mission_node` stops publishing formation-stage setpoints.
- `leader_control_node` controls the leader.
- `follower_formation_node` controls the follower UAVs.

This avoids multiple nodes sending competing Offboard setpoints to the same PX4 instance.

Follower yaw is not forced to the formation reference yaw by default. The formation position can still use the configured formation yaw/shape, while each vehicle can keep its current yaw unless a later yaw-control feature updates that behavior.

## Takeoff and landing

Start synchronized takeoff:

```bash
ros2 service call /formation/takeoff std_srvs/srv/Trigger "{}"
```

Land all UAVs:

```bash
ros2 service call /formation/land std_srvs/srv/Trigger "{}"
```

After emergency landing, the system may wait for ground confirmation depending on mission state.

## Leader command

External user input should be sent to:

```text
/formation/leader_input
```

Message type:

```text
formation/msg/FormationCommand
```

Example: move the leader toward east and south for 10 seconds while using column formation with 1.2 m spacing:

```bash
ros2 topic pub --once /formation/leader_input formation/msg/FormationCommand "{
  command: 1,
  velocity_east: 0.2,
  velocity_north: -0.2,
  velocity_up: 0.0,
  yaw_rate: 0.0,
  duration: 10.0,
  formation_type: 'column',
  spacing: 1.2
}"
```

Command values:

```text
HOLD = 0
MOVE = 1
SET_YAW = 2
MISSION_COMPLETE = 3
EMERGENCY_STOP = 4
```

`leader_command_node` receives `/formation/leader_input`, attaches the current leader id / leader generation, then republishes the active command to:

```text
/formation/command
```

The command is a high-level control intent. It is not a PX4 `VehicleCommand`, and it is not a pre-written waypoint path such as "fly forward 100 m then left 100 m".

## Leader and slot behavior

The leader can be changed with:

```bash
ros2 service call /formation/set_leader formation/srv/SetLeader "{leader_id: 1}"
```

In leader-follower mode, follower slot targets are calculated relative to the latest leader position.

The slot log shows relative offsets, not absolute world positions. Example:

```text
Slot assignment locked relative to latest leader position:
MAV1=(0.00, 0.00, 0.00), MAV2=(0.00, -1.20, 0.00), MAV3=(0.00, 1.20, 0.00)
```

This means:

```text
MAV1 is the leader/center slot
MAV2 is 1.2 m on one side
MAV3 is 1.2 m on the other side
```

When the leader moves, the anchor is updated from the leader's latest position, so the follower targets move with the leader.

## Formation shapes

Supported shape names:

- `line`
- `column`
- `triangle`
- `v_shape`

Current convention:

```text
line:
    lateral left/right formation, north-axis offsets when yaw = 0

column:
    forward/back formation, east-axis offsets when yaw = 0

triangle / v_shape:
    leader/front slot with two rear follower slots
```

Shape and spacing can be changed through `/formation/leader_input` by setting:

```yaml
formation_type: 'column'
spacing: 1.2
```

The follower node resets slot assignment when the leader, shape, or spacing changes.

## Formation complete

`follower_formation_node` publishes formation readiness through:

```text
/formation/follower_status
```

`mission_node` republishes the final mission status through:

```text
/formation/status
```

Formation is considered complete only when:

```text
maximum follower position error <= formation_position_tolerance
maximum follower speed <= formation_speed_tolerance
both conditions are held for formation_hold_duration seconds
```

These parameters are internal follower-node parameters and are not added to the msg definition.

Example:

```yaml
formation_position_tolerance: 0.5
formation_speed_tolerance: 0.15
formation_hold_duration: 2.0
```

## Configuration files

The main configuration is split into smaller files:

```text
config/mission.yaml
config/leader_command.yaml
config/leader_control.yaml
config/follower_formation.yaml
config/distributed.yaml
```

`config/formation.yaml` is kept as a package-level reference/index file.

Important mission parameters:

```yaml
control_frequency: 50.0
control_mode: leader_follower
leader_id: 1
formation_type: column
formation_spacing: 1.2
takeoff_height: 3.0
command_timeout: 1.0
```

Takeoff stabilization parameters:

```yaml
arm_retry_interval: 0.5
arm_retry_timeout: 8.0
liftoff_after_arm_timeout: 10.0
takeoff_climb_rate: 0.25
```

Meaning:

```text
arm_retry_interval:
    How often mission_node retries Offboard/Arm for pending UAVs during TAKEOFF.

arm_retry_timeout:
    How long to retry before reporting an Offboard/Arm timeout.

liftoff_after_arm_timeout:
    How long an already-armed UAV may hold at ground/home height while waiting for all UAVs to become armed/offboard.

takeoff_climb_rate:
    Maximum altitude setpoint ramp rate in m/s after liftoff is authorized.
```

Important follower formation parameters:

```yaml
formation_position_tolerance: 0.5
formation_speed_tolerance: 0.15
formation_hold_duration: 2.0
slot_assignment_lock: true
```

CPF avoidance parameters:

```yaml
cpf_enabled: true
cpf_attraction_gain: 0.5
cpf_repulsion_gain: 1.2
cpf_safe_distance: 0.8
cpf_influence_distance: 3.0
cpf_max_speed: 0.6
cpf_output_velocity: true
```

Safety parameters:

```yaml
minimum_vehicle_distance: 0.8
maximum_altitude: 20.0
maximum_speed: 3.0
geofence_radius: 50.0
maximum_setpoint_jump: 6.0
```

Rectangular real-flight fence parameters:

```yaml
fence_enabled: true
fence_world_x_min: -1.7
fence_world_x_max: 1.7
fence_world_y_min: -1.5
fence_world_y_max: 1.5
fence_height_min: 0.0
fence_height_max: 1.4
fence_brake_distance_m: 0.35
near_fence_margin_m: 0.3
```

In this package, the fence uses ROS ENU naming:

```text
x = east
y = north
z = up
```

`SafetyManager` uses the rectangular fence to detect out-of-bounds states and setpoints. `leader_control_node` and `follower_formation_node` also apply velocity braking near fence boundaries so commands slow down before reaching the wall.

For Gazebo profiles, `fence_enabled` is currently false by default because the simulation spawn offsets and takeoff altitude may exceed the small real-flight room fence. For real-flight profiles, `fence_enabled` is true.

Vehicle world origins must match the Gazebo spawn positions:

```yaml
vehicle_origins_enu: [
  0.0, 0.0, 0.0,
  2.0, 0.0, 0.0,
  0.0, 2.0, 0.0
]
```

These values are used to convert each PX4 local ENU frame into one shared Gazebo world ENU frame.

## DDS monitoring helper

A helper script is provided:

```bash
ros2 run formation monitor_mav_dds.sh
```

It checks whether required PX4 DDS topics exist for `MAV1`, `MAV2`, and `MAV3`.

Expected input topics include:

```text
/MAVx/fmu/in/offboard_control_mode
/MAVx/fmu/in/trajectory_setpoint
/MAVx/fmu/in/vehicle_command
```

Expected output topics include:

```text
/MAVx/fmu/out/vehicle_status_v1
/MAVx/fmu/out/vehicle_local_position_v1
```

The exact `_v1` suffix depends on the `px4_msgs` / PX4 version.

## Real-world OptiTrack direction

For real-world testing, the recommended architecture is:

```text
OptiTrack / Motive
    -> VRPN / ROS 2 pose topic
    -> mocap_px4_bridge, ENU to PX4-compatible odometry
    -> /MAVx/fmu/in/vehicle_visual_odometry
    -> PX4 EKF2
    -> /MAVx/fmu/out/vehicle_local_position_v1
    -> formation package
```

The formation controller should ideally continue using PX4 local position output. This keeps Gazebo and real-world operation consistent:

```text
Gazebo:
    simulated sensors / PX4 estimator -> PX4 local position -> formation

Real world:
    OptiTrack / external vision -> PX4 local position -> formation
```

The rectangular fence assumes that PX4 `vehicle_local_position_v1` is aligned with the shared OptiTrack/external-vision world frame. Before flight, verify this by moving each vehicle by hand and checking that `/MAVx/fmu/out/vehicle_local_position_v1` changes consistently with the real room axes and fence limits.

A future position-provider or mocap bridge can publish unified pose topics such as:

```text
/mocap/MAV1/pose
/mocap/MAV2/pose
/mocap/MAV3/pose
```

## Documentation

Code explanations are available in:

```text
docs/code_file_explanations/
```

Start from:

```text
docs/code_file_explanations/README.txt
```

These notes explain each source file, configuration file, launch file, msg, and srv interface.

## Notes

- Do not run multiple Offboard controllers for the same PX4 instance at the same time.
- In leader-follower mode, `mission_node` owns takeoff/landing, while `leader_control_node` and `follower_formation_node` own formation-stage movement.
- If Gazebo reports the vehicle is flying but the model does not move, check PX4 `gz_bridge`, Gazebo physics state, and motor command topics.
- If `/formation/takeoff` succeeds but the mission stays in `WAITING_READY`, check PX4 preflight, local position, DDS topics, and Micro XRCE-DDS Agent connection.
- If `/formation/leader_input` does not move the leader, check whether `/formation/status` has reached `WAITING_LEADER_COMMAND` or `FORMATION`, and verify `/formation/command` is being published.

## Recent development summary

Current updates:

- `formation.launch.py` can now accept external parameter files through launch arguments.
- Added `formation_real.launch.py` for real-world testing.
- Added `mission_real.yaml` for real-flight mission parameters.
- Added `follower_formation_real.yaml` for real-flight follower formation parameters.
- Real-flight YAML sets `vehicle_origins_enu` to zero, assuming PX4 local position is already in the shared OptiTrack / external-vision frame.
- Gazebo simulation keeps the original YAML files and spawn-origin offsets.
- Added Offboard/Arm retry logging for multi-PX4 startup reliability.
- Added armed/offboard liftoff barrier so armed vehicles hold ground/home height until the group is ready to climb.
- Added altitude ramp for takeoff setpoints through `takeoff_climb_rate`.
- Added rectangular ENU fence parameters and velocity braking near fence boundaries.

## TODO

- Add `rc_leader_input_node`:
  - Subscribe to `/MAV1/fmu/out/manual_control_setpoint` or configurable leader RC topic.
  - Convert RC pitch / roll / yaw input into `formation/msg/FormationCommand`.
  - Publish commands to `/formation/leader_input`.
  - Reuse the existing `leader_command_node -> leader_control_node -> follower_formation_node` pipeline.
  - Keep RC teleoperation as an optional real-flight feature, not enabled by default in the Gazebo launch.
- Add real-world OptiTrack bridge implementation after validating PX4 EKF2 external-vision parameters.
- Add position plausibility filtering similar to the C++ staged-testing node, for rejecting mocap/EKF jumps before they affect formation or fence calculations.
- Later split `follower_formation_node` into per-vehicle `follower_vehicle_node` for true distributed onboard execution.

## License

Apache-2.0
