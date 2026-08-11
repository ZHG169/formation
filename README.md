# formation

ROS 2 Humble package for PX4/Gazebo multi-UAV formation control.

This package is designed for three PX4 SITL UAVs with Micro XRCE-DDS communication. It supports synchronized takeoff and landing, centralized formation control, distributed formation control, leader command relay, ENU/NED coordinate conversion, and CPF velocity-based collision avoidance.

## Main features

- Three-UAV synchronized takeoff and landing
- PX4 Offboard control through `px4_msgs`
- Centralized formation control mode
- Distributed per-vehicle formation control mode
- Leader command topic: `/formation/command`
- Leader selection service
- Formation shape / spacing service
- ROS ENU to PX4 NED coordinate conversion
- Control Potential Field, CPF, velocity-based UAV avoidance
- Safety checks for distance, speed, altitude, geofence, timeout, and setpoint validity

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
│   └── formation.yaml
├── docs/
│   └── code_file_explanations/
├── formation/
│   ├── main_node.py
│   ├── mission_manager.py
│   ├── vehicle_interface.py
│   ├── formation_controller.py
│   ├── formation_shapes.py
│   ├── cpf_avoidance.py
│   ├── safety_manager.py
│   ├── leader_manager.py
│   ├── leader_command_node.py
│   ├── distributed_vehicle_node.py
│   └── coordinate_convert.py
├── launch/
│   ├── formation.launch.py
│   └── multi_PX4.launch.py
├── msg/
│   ├── FormationCommand.msg
│   ├── FormationError.msg
│   └── FormationStatus.msg
├── scripts/
│   ├── formation_node
│   ├── leader_command_node
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

The package assumes ROS/Gazebo side high-level logic uses ENU coordinates. PX4 Offboard setpoints are converted to NED before publishing.

## Run formation control

Centralized mode, default:

```bash
ros2 launch formation formation.launch.py
```

or explicitly:

```bash
ros2 launch formation formation.launch.py control_mode:=centralized
```

Distributed mode:

```bash
ros2 launch formation formation.launch.py control_mode:=distributed
```

In centralized mode, `formation_node` computes setpoints for all UAVs.

In distributed mode, `formation_node` manages mission state, while `distributed_vehicle_1`, `distributed_vehicle_2`, and `distributed_vehicle_3` compute per-vehicle setpoints during the FORMATION stage.

## Takeoff and landing

Start synchronized takeoff:

```bash
ros2 service call /formation/takeoff std_srvs/srv/Trigger "{}"
```

Land all UAVs:

```bash
ros2 service call /formation/land std_srvs/srv/Trigger "{}"
```

After landing or emergency landing, the system may wait for ground confirmation depending on mission state.

## Leader and formation services

Set leader:

```bash
ros2 service call /formation/set_leader formation/srv/SetLeader "{leader_id: 1}"
```

Set formation shape and spacing:

```bash
ros2 service call /formation/set_formation formation/srv/SetFormation "{formation_type: 'triangle', spacing: 2.0, control_mode: 'centralized'}"
```

Supported formation shapes are defined in `formation/formation_shapes.py`, currently including:

- `triangle`
- `line`
- `v_shape`
- `column`

## Leader command topic

The high-level command topic is:

```text
/formation/command
```

Message type:

```text
formation/msg/FormationCommand
```

The command is intended to represent high-level formation intent, such as hold, move, yaw update, mission complete, or emergency stop. It is not a PX4 `VehicleCommand`, and it is not a pre-written waypoint path.

Main command values:

```text
HOLD = 0
MOVE = 1
SET_YAW = 2
MISSION_COMPLETE = 3
EMERGENCY_STOP = 4
```

The leader command relay node publishes `/formation/command` and listens to `/formation/leader_input`.

## Configuration

Main parameters are in:

```text
config/formation.yaml
```

Important parameters:

```yaml
control_frequency: 100.0
control_mode: centralized
leader_id: 1
formation_type: triangle
formation_spacing: 2.0
takeoff_height: 3.0
formation_reference_enu: [4.0, 0.0, 3.0]
formation_reference_yaw_enu: 0.0
command_timeout: 1.0
```

CPF avoidance parameters:

```yaml
cpf_enabled: true
cpf_attraction_gain: 0.5
cpf_repulsion_gain: 1.2
cpf_safe_distance: 1.5
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
- In distributed mode, the ground formation node skips formation-stage heartbeat/setpoint publishing so the per-vehicle distributed nodes can control their own UAVs.
- If Gazebo reports the vehicle is flying but the model does not move, check PX4 `gz_bridge`, Gazebo physics state, and motor command topics.
- If `/formation/takeoff` succeeds but the mission stays in `WAITING_READY`, check PX4 preflight, local position, DDS topics, and Micro XRCE-DDS Agent connection.

## License

Apache-2.0
