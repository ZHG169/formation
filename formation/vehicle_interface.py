from dataclasses import dataclass
from typing import Optional
import math

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from formation.coordinate_convert import (
    VectorENU,
    VectorNED,
    enu_to_ned,
    ned_to_enu,
    yaw_enu_to_ned,
    yaw_ned_to_enu,
)

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

@dataclass
class VehicleState:
    vehicle_id: int
    namespace: str

    position_local_enu: Optional[VectorENU] = None
    velocity_local_enu: Optional[VectorENU] = None
    yaw_local_enu: float = 0.0

    armed: bool = False
    landed: bool = False
    preflight_ok: bool = False
    position_valid: bool = False

    status_received: bool = False
    position_received: bool = False
    land_status_received: bool = False

    last_position_update_ns: int = 0

    nav_state: int = 0
    offboard_enabled: bool = False


@dataclass
class VehicleSetpoint:
    position_local_enu: VectorENU
    yaw_local_enu: float
    velocity_local_enu: Optional[VectorENU] = None


class VehicleInterface:

    def __init__(self, node, namespace, system_id):
        self.node = node
        self.namespace = namespace
        self.system_id = system_id
        self.state = VehicleState(
            vehicle_id=system_id,
            namespace=namespace,
        )

        input_prefix = f'/{namespace}/fmu/in'
        output_prefix = f'/{namespace}/fmu/out'
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_publisher = (
            self.node.create_publisher(
                OffboardControlMode,
                f'{input_prefix}/offboard_control_mode',
                qos,
            )
        )

        self.setpoint_publisher = (
            self.node.create_publisher(
                TrajectorySetpoint,
                f'{input_prefix}/trajectory_setpoint',
                qos,
            )
        )

        self.command_publisher = (
            self.node.create_publisher(
                VehicleCommand,
                f'{input_prefix}/vehicle_command',
                qos,
            )
        )

        self.status_subscription = (
            self.node.create_subscription(
                VehicleStatus,
                f'{output_prefix}/vehicle_status_v1',
                self.status_callback,
                qos,
            )
        )

        self.position_subscription = (
            self.node.create_subscription(
                VehicleLocalPosition,
                f'{output_prefix}/vehicle_local_position_v1',
                self.position_callback,
                qos,
            )
        )

        self.land_subscription = (
            self.node.create_subscription(
                VehicleLandDetected,
                f'{output_prefix}/vehicle_land_detected',
                self.land_detected_callback,
                qos,
            )
        )


        # 在此建立 publishers 與 subscriptions。

    def status_callback(self, message):
        self.state.status_received = True
        self.state.preflight_ok = bool(
            message.pre_flight_checks_pass
        )

        self.state.armed = (
            message.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

        self.state.nav_state = int(message.nav_state)

        self.state.offboard_enabled = (
            message.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def position_is_fresh(
        self,
        timeout_seconds: float = 2.0,
    ) -> bool:
        if not self.state.position_received:
            return False

        now_ns = self.node.get_clock().now().nanoseconds

        age_seconds = (
            now_ns - self.state.last_position_update_ns
        ) / 1e9

        return bool(
            self.state.position_valid
            and 0.0 <= age_seconds <= timeout_seconds
        )

    def position_callback(self, message):
        self.state.position_received = True

        values_finite = all(
            math.isfinite(float(value))
            for value in (
                message.x,
                message.y,
                message.z,
                message.vx,
                message.vy,
                message.vz,
                message.heading,
            )
        )

        self.state.position_valid = bool(
            message.xy_valid
            and message.z_valid
            and message.v_xy_valid
            and message.v_z_valid
            and values_finite
        )

        self.state.last_position_update_ns = (
            self.node.get_clock().now().nanoseconds
        )

        if not self.state.position_valid:
            self.state.position_local_enu = None
            self.state.velocity_local_enu = None
            return

        position_ned = VectorNED(
            north=float(message.x),
            east=float(message.y),
            down=float(message.z),
        )

        self.state.position_local_enu = ned_to_enu(
            position_ned
        )

        velocity_ned = VectorNED(
            north=float(message.vx),
            east=float(message.vy),
            down=float(message.vz),
        )

        self.state.velocity_local_enu = ned_to_enu(
            velocity_ned
        )

        self.state.yaw_local_enu = yaw_ned_to_enu(
            float(message.heading)
        )

    def get_state(self):
        return self.state

    def publish_offboard_heartbeat(self, use_velocity=False):
        message = OffboardControlMode()

        message.timestamp = self.timestamp_us()
        message.position = not bool(use_velocity)
        message.velocity = bool(use_velocity)
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False

        self.offboard_publisher.publish(message)

    def publish_setpoint(self, setpoint):
        message = TrajectorySetpoint()
        message.timestamp = self.timestamp_us()
        message.yaw = yaw_enu_to_ned(setpoint.yaw_local_enu)

        if setpoint.velocity_local_enu is not None:
            velocity_ned = enu_to_ned(setpoint.velocity_local_enu)
            message.position = [
                math.nan,
                math.nan,
                math.nan,
            ]
            message.velocity = velocity_ned.as_list()
        else:
            target_ned = enu_to_ned(setpoint.position_local_enu)
            message.position = target_ned.as_list()
            message.velocity = [
                math.nan,
                math.nan,
                math.nan,
            ]

        self.setpoint_publisher.publish(message)
    
    def set_offboard_mode(self):
        self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def arm(self):
        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    def disarm(self):
        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )

    def land_detected_callback(self, message):
        self.state.land_status_received = True
        self.state.landed = bool(message.landed)

    def land(self):
        self.send_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

    def send_command(self, command, param1=0.0, param2=0.0):
        message = VehicleCommand()
        message.timestamp = self.timestamp_us()
        message.command = command

        message.param1 = float(param1)
        message.param2 = float(param2)

        message.target_system = self.system_id
        message.target_component = 1

        message.source_system = 255
        message.source_component = 1
        message.from_external = True

        self.command_publisher.publish(message)

    def timestamp_us(self):
        return self.node.get_clock().now().nanoseconds // 1000