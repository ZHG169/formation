import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from formation.coordinate_convert import VectorENU
from formation.cpf_avoidance import CpfConfig, limit_velocity_near_fence
from formation.msg import FormationCommand, FormationStatus
from formation.mission_manager import MissionState
from formation.vehicle_interface import VehicleInterface, VehicleSetpoint


CONTROL_MODE_LEADER_FOLLOWER = 'leader_follower'


class LeaderControlNode(Node):
    """Control only the leader UAV during FORMATION.

    mission_node owns synchronized takeoff and landing.  This node starts
    publishing leader Offboard heartbeat/setpoints only after the mission
    state becomes FORMATION.
    """

    def __init__(self):
        super().__init__('leader_control_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_frequency', 50.0),
                ('leader_id', 1),
                ('command_timeout', 1.0),
                ('yaw_debug_enabled', False),
                ('yaw_debug_interval', 1.0),
                ('cpf_max_speed', 0.6),
                ('fence_enabled', False),
                ('fence_world_x_min', -50.0),
                ('fence_world_x_max', 50.0),
                ('fence_world_y_min', -50.0),
                ('fence_world_y_max', 50.0),
                ('fence_height_min', 0.0),
                ('fence_height_max', 20.0),
                ('fence_brake_distance_m', 0.35),
                ('vehicle_names', ['MAV1', 'MAV2', 'MAV3']),
                ('vehicle_system_ids', [1, 2, 3]),
            ],
        )

        frequency = float(
            self.get_parameter('control_frequency').value
        )
        if frequency <= 2.0:
            raise ValueError(
                'control_frequency must be greater than 2 Hz'
            )

        names = list(self.get_parameter('vehicle_names').value)
        system_ids = [
            int(value)
            for value in self.get_parameter(
                'vehicle_system_ids'
            ).value
        ]
        if len(names) != len(system_ids):
            raise ValueError(
                'vehicle_names and vehicle_system_ids length mismatch'
            )

        self.vehicles = {
            system_id: VehicleInterface(self, name, system_id)
            for name, system_id in zip(names, system_ids)
        }

        self.active_leader_id = int(
            self.get_parameter('leader_id').value
        )
        self.leader_generation = 0
        self.mission_state = MissionState.IDLE.name
        self.control_mode = ''
        self.latest_command = None
        self.last_command_receive_ns = 0
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        self.hold_yaw_enu = 0.0
        self.fence_config = CpfConfig(
            enabled=False,
            max_speed=float(
                self.get_parameter('cpf_max_speed').value
            ),
            fence_enabled=bool(
                self.get_parameter('fence_enabled').value
            ),
            fence_world_x_min=float(
                self.get_parameter('fence_world_x_min').value
            ),
            fence_world_x_max=float(
                self.get_parameter('fence_world_x_max').value
            ),
            fence_world_y_min=float(
                self.get_parameter('fence_world_y_min').value
            ),
            fence_world_y_max=float(
                self.get_parameter('fence_world_y_max').value
            ),
            fence_height_min=float(
                self.get_parameter('fence_height_min').value
            ),
            fence_height_max=float(
                self.get_parameter('fence_height_max').value
            ),
            fence_brake_distance_m=float(
                self.get_parameter('fence_brake_distance_m').value
            ),
        )

        self.status_subscription = self.create_subscription(
            FormationStatus,
            '/formation/status',
            self.status_callback,
            10,
        )
        self.command_subscription = self.create_subscription(
            FormationCommand,
            '/formation/command',
            self.command_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / frequency,
            self.control_loop,
        )

        self.get_logger().info(
            f'Leader control node ready at {frequency:.1f} Hz'
        )

    def status_callback(self, message):
        leader_changed = (
            int(message.leader_id) != self.active_leader_id
            or int(message.leader_generation)
            != self.leader_generation
        )

        self.active_leader_id = int(message.leader_id)
        self.leader_generation = int(message.leader_generation)
        self.mission_state = str(message.mission_state)
        self.control_mode = str(message.control_mode)

        if leader_changed:
            self.latest_command = None
            self.last_command_receive_ns = 0

    def command_callback(self, message):
        if int(message.leader_id) != self.active_leader_id:
            return
        if int(message.leader_generation) != self.leader_generation:
            return

        self.latest_command = message
        self.last_command_receive_ns = (
            self.get_clock().now().nanoseconds
        )

    def formation_active(self):
        return bool(
            self.mission_state == MissionState.FORMATION.name
            and self.control_mode == CONTROL_MODE_LEADER_FOLLOWER
        )

    def command_is_fresh(self, now_ns):
        if self.latest_command is None:
            return False

        age_seconds = (
            now_ns - self.last_command_receive_ns
        ) / 1e9
        return 0.0 <= age_seconds <= self.command_timeout

    def control_loop(self):
        if not self.formation_active():
            return

        vehicle = self.vehicles.get(self.active_leader_id)
        if vehicle is None:
            return

        state = vehicle.get_state()
        if not state.position_valid or state.position_local_enu is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        command = (
            self.latest_command
            if self.command_is_fresh(now_ns)
            else None
        )

        use_velocity = False
        setpoint = VehicleSetpoint(
            position_local_enu=state.position_local_enu,
            yaw_local_enu=state.yaw_local_enu,
        )

        if command is not None:
            if command.command == FormationCommand.MOVE:
                velocity = VectorENU(
                    east=float(command.velocity_east),
                    north=float(command.velocity_north),
                    up=float(command.velocity_up),
                )
                velocity = limit_velocity_near_fence(
                    state.position_local_enu,
                    velocity,
                    self.fence_config,
                )
                setpoint = VehicleSetpoint(
                    position_local_enu=state.position_local_enu,
                    yaw_local_enu=state.yaw_local_enu,
                    velocity_local_enu=velocity,
                )
                use_velocity = True
            elif command.command == FormationCommand.SET_YAW:
                self.hold_yaw_enu = float(command.yaw_enu)
                setpoint = VehicleSetpoint(
                    position_local_enu=state.position_local_enu,
                    yaw_local_enu=self.hold_yaw_enu,
                )

        vehicle.publish_offboard_heartbeat(use_velocity=use_velocity)
        vehicle.publish_setpoint(setpoint)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderControlNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
