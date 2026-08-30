import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from formation.coordinate_convert import VectorENU
from formation.fence_limiter import (
    FenceConfig,
    limit_velocity_near_fence,
)
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
                ('target_debug_enabled', False),
                ('target_debug_interval', 1.0),
                ('cpf_max_speed', 0.6),
                ('formation_envelope_enabled', True),
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
        self.active_move_sequence = None
        self.move_start_ns = 0
        self.move_duration = 0.0
        self.formation_offsets = None
        self.formation_envelope_enabled = bool(
            self.get_parameter('formation_envelope_enabled').value
        )
        self.hold_yaw_enu = 0.0
        self.last_target_debug_ns = 0
        self.fence_config = FenceConfig(
            enabled=bool(
                self.get_parameter('fence_enabled').value
            ),
            world_x_min=float(
                self.get_parameter('fence_world_x_min').value
            ),
            world_x_max=float(
                self.get_parameter('fence_world_x_max').value
            ),
            world_y_min=float(
                self.get_parameter('fence_world_y_min').value
            ),
            world_y_max=float(
                self.get_parameter('fence_world_y_max').value
            ),
            height_min=float(
                self.get_parameter('fence_height_min').value
            ),
            height_max=float(
                self.get_parameter('fence_height_max').value
            ),
            brake_distance_m=float(
                self.get_parameter('fence_brake_distance_m').value
            ),
            max_speed=float(
                self.get_parameter('cpf_max_speed').value
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
            self.active_move_sequence = None
            self.move_start_ns = 0
            self.move_duration = 0.0
            self.formation_offsets = None

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

    def command_sequence(self, command):
        return int(getattr(command, 'sequence', 0))

    def start_move_if_needed(self, command, now_ns):
        sequence = self.command_sequence(command)
        if self.active_move_sequence == sequence:
            return

        self.active_move_sequence = sequence
        self.move_start_ns = now_ns
        self.move_duration = max(float(command.duration), 0.0)

        self.get_logger().info(
            f'Leader MOVE sequence {sequence} started for '
            f'{self.move_duration:.2f} s'
        )

    def move_is_active(self, now_ns):
        if self.active_move_sequence is None:
            return False

        elapsed = (now_ns - self.move_start_ns) / 1e9
        return 0.0 <= elapsed <= self.move_duration

    def subtract_vectors(self, first, second):
        return VectorENU(
            east=first.east - second.east,
            north=first.north - second.north,
            up=first.up - second.up,
        )

    def lock_formation_offsets_if_needed(self):
        if not self.formation_envelope_enabled:
            return
        if self.formation_offsets is not None:
            return

        positions = {}
        for vehicle_id, vehicle in self.vehicles.items():
            state = vehicle.get_state()
            if not state.position_valid:
                return
            if state.position_local_enu is None:
                return
            positions[vehicle_id] = state.position_local_enu

        if self.active_leader_id not in positions:
            return

        leader_position = positions[self.active_leader_id]
        self.formation_offsets = {
            vehicle_id: self.subtract_vectors(
                position,
                leader_position,
            )
            for vehicle_id, position in positions.items()
        }

        log_message = (
            'Leader formation envelope locked from current offsets: '
            + ', '.join(
                f'MAV{vehicle_id}=({offset.east:.2f}, '
                f'{offset.north:.2f}, {offset.up:.2f})'
                for vehicle_id, offset
                in sorted(self.formation_offsets.items())
            )
        )

        if self.fence_config.enabled:
            envelope = self.formation_envelope_config()
            log_message += (
                ' | leader safe range: '
                f'x=[{envelope.world_x_min:.2f}, '
                f'{envelope.world_x_max:.2f}], '
                f'y=[{envelope.world_y_min:.2f}, '
                f'{envelope.world_y_max:.2f}]'
            )

        self.get_logger().info(log_message)

    def formation_envelope_config(self):
        if (
            not self.formation_envelope_enabled
            or not self.fence_config.enabled
            or self.formation_offsets is None
        ):
            return self.fence_config

        min_offset_east = min(
            offset.east for offset in self.formation_offsets.values()
        )
        max_offset_east = max(
            offset.east for offset in self.formation_offsets.values()
        )
        min_offset_north = min(
            offset.north for offset in self.formation_offsets.values()
        )
        max_offset_north = max(
            offset.north for offset in self.formation_offsets.values()
        )

        leader_x_min = self.fence_config.world_x_min - min_offset_east
        leader_x_max = self.fence_config.world_x_max - max_offset_east
        leader_y_min = self.fence_config.world_y_min - min_offset_north
        leader_y_max = self.fence_config.world_y_max - max_offset_north

        if leader_x_min > leader_x_max or leader_y_min > leader_y_max:
            self.get_logger().error(
                'Formation is larger than the configured fence; '
                'falling back to leader-only fence limit'
            )
            return self.fence_config

        return FenceConfig(
            enabled=True,
            world_x_min=leader_x_min,
            world_x_max=leader_x_max,
            world_y_min=leader_y_min,
            world_y_max=leader_y_max,
            height_min=self.fence_config.height_min,
            height_max=self.fence_config.height_max,
            brake_distance_m=self.fence_config.brake_distance_m,
            max_speed=self.fence_config.max_speed,
        )

    def log_leader_target_debug(self, state, velocity, command, now_ns):
        enabled = bool(
            self.get_parameter('target_debug_enabled').value
        )
        if not enabled:
            return

        interval = float(
            self.get_parameter('target_debug_interval').value
        )
        if now_ns - self.last_target_debug_ns < interval * 1e9:
            return

        self.last_target_debug_ns = now_ns
        current = state.position_local_enu
        duration = max(float(command.duration), 0.0)
        predicted = VectorENU(
            east=current.east + velocity.east * duration,
            north=current.north + velocity.north * duration,
            up=current.up + velocity.up * duration,
        )
        self.get_logger().info(
            f'Leader target debug MAV{self.active_leader_id}: '
            f'current_local_enu=({current.east:.2f}, '
            f'{current.north:.2f}, {current.up:.2f}), '
            f'velocity_cmd_enu=({velocity.east:.2f}, '
            f'{velocity.north:.2f}, {velocity.up:.2f}), '
            f'duration={duration:.2f}s, '
            f'predicted_end_local_enu=({predicted.east:.2f}, '
            f'{predicted.north:.2f}, {predicted.up:.2f})'
        )

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

        self.lock_formation_offsets_if_needed()

        use_velocity = True
        velocity = VectorENU(0.0, 0.0, 0.0)
        yaw = state.yaw_local_enu

        if command is not None:
            if command.command == FormationCommand.MOVE:
                self.start_move_if_needed(command, now_ns)
                requested_velocity = VectorENU(
                    east=float(command.velocity_east),
                    north=float(command.velocity_north),
                    up=float(command.velocity_up),
                )
                velocity = limit_velocity_near_fence(
                    state.position_local_enu,
                    requested_velocity,
                    self.fence_config,
                )
                velocity = self.limit_velocity_by_formation_envelope(
                    state.position_local_enu,
                    velocity,
                )
                self.log_leader_target_debug(
                    state,
                    velocity,
                    command,
                    now_ns,
                )
                if not self.move_is_active(now_ns):
                    velocity = VectorENU(0.0, 0.0, 0.0)
            elif command.command == FormationCommand.SET_YAW:
                self.hold_yaw_enu = float(command.yaw_enu)
                yaw = self.hold_yaw_enu

        setpoint = VehicleSetpoint(
            position_local_enu=state.position_local_enu,
            yaw_local_enu=yaw,
            velocity_local_enu=velocity,
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
