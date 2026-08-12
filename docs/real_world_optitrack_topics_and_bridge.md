# Real-world OptiTrack / ROS 2 / PX4 topic plan

This note records the ROS 2 topics currently visible from PX4 and the planned data path for real-world OptiTrack positioning.

The goal is to keep the formation controller unchanged:

```text
PX4 local position -> formation -> PX4 offboard setpoints
```

OptiTrack is used only as an external positioning source for PX4 EKF2.

## Target architecture

```text
OptiTrack / Motive
    │ VRPN
    ▼
Ground station (.121)
├── vrpn_mocap
│       ├── /vrpn_mocap/MAV1/pose
│       ├── /vrpn_mocap/MAV2/pose
│       └── /vrpn_mocap/MAV3/pose
│
├── qos_relay.py
│       ├── /vrpn_mocap/MAV1/pose_reliable
│       ├── /vrpn_mocap/MAV2/pose_reliable
│       └── /vrpn_mocap/MAV3/pose_reliable
│
└── mocap_px4_bridge
        ├── /MAV1/fmu/in/vehicle_visual_odometry
        ├── /MAV2/fmu/in/vehicle_visual_odometry
        └── /MAV3/fmu/in/vehicle_visual_odometry

ROS 2 DDS, domain 42, FastDDS unicast
    │
    ├── Pi MAV1 (.177)
    │       MicroXRCEAgent -n MAV1
    │       serial 921600
    │       Pix32 v6, MAV_SYS_ID=1, UXRCE_DDS_KEY=1
    │
    ├── Pi MAV2 (.153)
    │       MicroXRCEAgent -n MAV2
    │       serial 921600
    │       Pix32 v6, MAV_SYS_ID=2, UXRCE_DDS_KEY=2
    │
    └── Pi MAV3
            MicroXRCEAgent -n MAV3
            serial 921600
            Pix32 v6, MAV_SYS_ID=3, UXRCE_DDS_KEY=3
```

## Important concept

Formation does not directly use OptiTrack pose.

The correct flow is:

```text
OptiTrack pose
    -> PX4 external vision / mocap input
    -> PX4 EKF2 local position
    -> /MAVx/fmu/out/vehicle_local_position_v1
    -> formation package
```

So the formation package continues to subscribe to:

```text
/MAV1/fmu/out/vehicle_local_position_v1
/MAV2/fmu/out/vehicle_local_position_v1
/MAV3/fmu/out/vehicle_local_position_v1
```

## PX4 topics recorded from current ROS 2 topic list

The current environment exposes these useful PX4 topics for each namespace:

```text
/MAVx/fmu/in/offboard_control_mode
/MAVx/fmu/in/trajectory_setpoint
/MAVx/fmu/in/vehicle_command
/MAVx/fmu/in/vehicle_visual_odometry
/MAVx/fmu/in/vehicle_mocap_odometry

/MAVx/fmu/out/vehicle_local_position_v1
/MAVx/fmu/out/vehicle_status_v1
/MAVx/fmu/out/vehicle_land_detected
/MAVx/fmu/out/vehicle_attitude
/MAVx/fmu/out/vehicle_odometry
/MAVx/fmu/out/vehicle_control_mode
/MAVx/fmu/out/vehicle_command_ack
/MAVx/fmu/out/estimator_status_flags
/MAVx/fmu/out/failsafe_flags
/MAVx/fmu/out/timesync_status
/MAVx/fmu/out/battery_status_v1
```

### Formation control input topics

These are published by the formation package to PX4:

```text
/MAV1/fmu/in/offboard_control_mode
/MAV1/fmu/in/trajectory_setpoint
/MAV1/fmu/in/vehicle_command

/MAV2/fmu/in/offboard_control_mode
/MAV2/fmu/in/trajectory_setpoint
/MAV2/fmu/in/vehicle_command

/MAV3/fmu/in/offboard_control_mode
/MAV3/fmu/in/trajectory_setpoint
/MAV3/fmu/in/vehicle_command
```

### PX4 state output topics used by formation

These are subscribed by `VehicleInterface`:

```text
/MAV1/fmu/out/vehicle_local_position_v1
/MAV1/fmu/out/vehicle_status_v1
/MAV1/fmu/out/vehicle_land_detected

/MAV2/fmu/out/vehicle_local_position_v1
/MAV2/fmu/out/vehicle_status_v1
/MAV2/fmu/out/vehicle_land_detected

/MAV3/fmu/out/vehicle_local_position_v1
/MAV3/fmu/out/vehicle_status_v1
/MAV3/fmu/out/vehicle_land_detected
```

### OptiTrack / mocap input topics to PX4

Recommended first target:

```text
/MAV1/fmu/in/vehicle_visual_odometry
/MAV2/fmu/in/vehicle_visual_odometry
/MAV3/fmu/in/vehicle_visual_odometry
```

Alternative available topic:

```text
/MAV1/fmu/in/vehicle_mocap_odometry
/MAV2/fmu/in/vehicle_mocap_odometry
/MAV3/fmu/in/vehicle_mocap_odometry
```

Both topics use `px4_msgs/msg/VehicleOdometry` in the current PX4 message set.

## OptiTrack side expected topics

The VRPN node should publish `geometry_msgs/msg/PoseStamped` topics such as:

```text
/vrpn_mocap/MAV1/pose
/vrpn_mocap/MAV2/pose
/vrpn_mocap/MAV3/pose
```

If QoS compatibility is an issue, relay them to reliable topics:

```text
/vrpn_mocap/MAV1/pose_reliable
/vrpn_mocap/MAV2/pose_reliable
/vrpn_mocap/MAV3/pose_reliable
```

## QoS relay concept

Some VRPN pose publishers use BEST_EFFORT QoS. A bridge node can be easier to debug if it subscribes to a reliable topic. The relay does:

```text
subscribe  /vrpn_mocap/MAVx/pose           BEST_EFFORT
publish    /vrpn_mocap/MAVx/pose_reliable  RELIABLE
```

Minimal structure:

```python
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# subscribe: /vrpn_mocap/MAV1/pose using best_effort_qos
# publish:   /vrpn_mocap/MAV1/pose_reliable using reliable_qos
```

## mocap_px4_bridge concept

The bridge converts:

```text
geometry_msgs/msg/PoseStamped
    -> px4_msgs/msg/VehicleOdometry
```

Input:

```text
/vrpn_mocap/MAV1/pose_reliable
/vrpn_mocap/MAV2/pose_reliable
/vrpn_mocap/MAV3/pose_reliable
```

Output:

```text
/MAV1/fmu/in/vehicle_visual_odometry
/MAV2/fmu/in/vehicle_visual_odometry
/MAV3/fmu/in/vehicle_visual_odometry
```

### Coordinate convention

PX4 `VehicleOdometry` can use NED frame:

```text
pose_frame = VehicleOdometry.POSE_FRAME_NED
velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
```

If the incoming pose is ROS ENU:

```text
ENU position:
    x = east
    y = north
    z = up

NED position:
    x = north = ENU.y
    y = east  = ENU.x
    z = down  = -ENU.z
```

So position conversion is:

```python
odom.position = [
    pose.pose.position.y,
    pose.pose.position.x,
    -pose.pose.position.z,
]
```

Orientation conversion must be verified with the real OptiTrack rigid body axes and PX4 body frame convention. Do not assume the quaternion is correct until the yaw/axis direction is checked in QGroundControl and PX4 logs.

## Rough mocap_px4_bridge Python structure

This is a planning skeleton, not final flight-ready code.

```python
#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry


class MocapPx4Bridge(Node):

    def __init__(self):
        super().__init__('mocap_px4_bridge')

        self.vehicle_names = ['MAV1', 'MAV2', 'MAV3']

        self.input_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.output_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publishers = {}
        self.subscriptions = []

        for name in self.vehicle_names:
            self.publishers[name] = self.create_publisher(
                VehicleOdometry,
                f'/{name}/fmu/in/vehicle_visual_odometry',
                self.output_qos,
            )

            subscription = self.create_subscription(
                PoseStamped,
                f'/vrpn_mocap/{name}/pose_reliable',
                lambda msg, vehicle=name: self.pose_callback(
                    vehicle,
                    msg,
                ),
                self.input_qos,
            )
            self.subscriptions.append(subscription)

    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def pose_callback(self, vehicle_name, pose):
        message = VehicleOdometry()
        timestamp = self.timestamp_us()

        message.timestamp = timestamp
        message.timestamp_sample = timestamp

        message.pose_frame = VehicleOdometry.POSE_FRAME_NED
        message.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # If OptiTrack pose is converted into ROS ENU before this node:
        message.position = [
            float(pose.pose.position.y),
            float(pose.pose.position.x),
            float(-pose.pose.position.z),
        ]

        # TODO: Replace this with verified ENU/OptiTrack -> PX4 NED quaternion conversion.
        # VehicleOdometry expects quaternion order [w, x, y, z].
        message.q = [
            float(pose.pose.orientation.w),
            float(pose.pose.orientation.x),
            float(pose.pose.orientation.y),
            float(pose.pose.orientation.z),
        ]

        nan = float('nan')
        message.velocity = [nan, nan, nan]
        message.angular_velocity = [nan, nan, nan]

        message.position_variance = [0.01, 0.01, 0.01]
        message.orientation_variance = [0.01, 0.01, 0.01]
        message.velocity_variance = [nan, nan, nan]

        message.reset_counter = 0
        message.quality = 100

        self.publishers[vehicle_name].publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MocapPx4Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Test commands

### 1. Check VRPN pose

```bash
ros2 topic echo --once /vrpn_mocap/MAV1/pose
ros2 topic echo --once /vrpn_mocap/MAV2/pose
ros2 topic echo --once /vrpn_mocap/MAV3/pose
```

### 2. Check reliable relay output

```bash
ros2 topic echo --once /vrpn_mocap/MAV1/pose_reliable
ros2 topic echo --once /vrpn_mocap/MAV2/pose_reliable
ros2 topic echo --once /vrpn_mocap/MAV3/pose_reliable
```

### 3. Check bridge output to PX4

```bash
ros2 topic echo --once /MAV1/fmu/in/vehicle_visual_odometry
ros2 topic echo --once /MAV2/fmu/in/vehicle_visual_odometry
ros2 topic echo --once /MAV3/fmu/in/vehicle_visual_odometry
```

### 4. Check PX4 estimator output

```bash
ros2 topic echo --once /MAV1/fmu/out/vehicle_local_position_v1
ros2 topic echo --once /MAV2/fmu/out/vehicle_local_position_v1
ros2 topic echo --once /MAV3/fmu/out/vehicle_local_position_v1
```

If step 3 works but step 4 does not, the problem is likely PX4 EKF2 external vision configuration, frame conversion, timestamping, or XRCE-DDS routing.

### 5. Check formation readiness topics

```bash
ros2 topic echo --once /formation/status
ros2 topic echo --once /formation/follower_status
```

## Real-world bring-up checklist

For each vehicle:

```text
MAV1:
    MAV_SYS_ID = 1
    UXRCE_DDS_KEY = 1
    ROS namespace = /MAV1

MAV2:
    MAV_SYS_ID = 2
    UXRCE_DDS_KEY = 2
    ROS namespace = /MAV2

MAV3:
    MAV_SYS_ID = 3
    UXRCE_DDS_KEY = 3
    ROS namespace = /MAV3
```

Before takeoff:

```text
1. OptiTrack rigid body is visible in Motive.
2. VRPN pose topic publishes for each UAV.
3. Reliable relay topic publishes for each UAV.
4. mocap_px4_bridge publishes vehicle_visual_odometry for each UAV.
5. PX4 EKF2 produces vehicle_local_position_v1 for each UAV.
6. formation package sees all vehicle_status_v1 and vehicle_local_position_v1 topics.
7. QGroundControl preflight checks are OK.
```

Only after these checks should `/formation/takeoff` be used.
