import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from px4_msgs.msg import VehicleLocalPosition

from formation.coordinate_convert import (
    VectorENU,
    VectorNED,
    ned_to_enu,
)
from formation.cpf_avoidance import CpfConfig, apply_cpf_to_setpoints
from formation.formation_controller import (
    FormationReference,
    add_vectors,
    advance_reference,
    rotate_offset,
    set_reference_yaw,
    subtract_vectors,
)
from formation.formation_shapes import get_shape
from formation.msg import FormationCommand, FormationStatus
from formation.vehicle_interface import (
    VehicleInterface,
    VehicleSetpoint,
    VehicleState,
)


class DistributedVehicleNode(Node):
    """Compute one vehicle setpoint from leader command only."""

    def __init__(self):
        super().__init__('distributed_vehicle_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_frequency', 100.0),
                ('vehicle_id', 1),
                ('namespace', 'MAV1'),
                ('vehicle_origin_enu', [0.0, 0.0, 0.0]),
                ('vehicle_ids', [1, 2, 3]),
                ('vehicle_names', ['MAV1', 'MAV2', 'MAV3']),
                ('vehicle_origins_enu', [
                    0.0, 0.0, 0.0,
                    2.0, 0.0, 0.0,
                    0.0, 2.0, 0.0,
                ]),
                ('formation_type', 'triangle'),
                ('formation_spacing', 2.0),
                ('formation_reference_enu', [0.0, 0.0, 3.0]),
                ('formation_reference_yaw_enu', 0.0),
                ('command_timeout', 1.0),
                ('local_position_timeout', 2.0),
                ('cpf_enabled', False),
                ('cpf_attraction_gain', 0.8),
                ('cpf_repulsion_gain', 1.2),
                ('cpf_safe_distance', 1.5),
                ('cpf_influence_distance', 4.0),
                ('cpf_max_speed', 0.8),
                ('cpf_output_velocity', True),
            ],
        )

        self.vehicle_id = int(
            self.get_parameter('vehicle_id').value
        )
        namespace = str(
            self.get_parameter('namespace').value
        )
        self.vehicle = VehicleInterface(
            self,
            namespace,
            self.vehicle_id,
        )

        origin = list(
            self.get_parameter('vehicle_origin_enu').value
        )
        reference = list(
            self.get_parameter('formation_reference_enu').value
        )
        if len(origin) != 3:
            raise ValueError(
                'vehicle_origin_enu must contain east, north, up'
            )
        if len(reference) != 3:
            raise ValueError(
                'formation_reference_enu must contain east, north, up'
            )

        self.vehicle_origin = VectorENU(
            east=float(origin[0]),
            north=float(origin[1]),
            up=float(origin[2]),
        )
        self.formation_reference = FormationReference(
            position_world_enu=VectorENU(
                east=float(reference[0]),
                north=float(reference[1]),
                up=float(reference[2]),
            ),
            yaw_enu=float(
                self.get_parameter(
                    'formation_reference_yaw_enu'
                ).value
            ),
        )

        self.vehicle_ids = [
            int(vehicle_id)
            for vehicle_id
            in self.get_parameter('vehicle_ids').value
        ]
        self.vehicle_names = [
            str(vehicle_name)
            for vehicle_name
            in self.get_parameter('vehicle_names').value
        ]
        origin_values = list(
            self.get_parameter('vehicle_origins_enu').value
        )
        if len(origin_values) != 3 * len(self.vehicle_ids):
            raise ValueError(
                'vehicle_origins_enu must contain three values '
                'for each vehicle_id'
            )
        if len(self.vehicle_names) != len(self.vehicle_ids):
            raise ValueError(
                'vehicle_names length must match vehicle_ids'
            )
        self.vehicle_origins = {
            vehicle_id: VectorENU(
                east=float(origin_values[index]),
                north=float(origin_values[index + 1]),
                up=float(origin_values[index + 2]),
            )
            for index, vehicle_id in zip(
                range(0, len(origin_values), 3),
                self.vehicle_ids,
            )
        }
        self.names_by_id = dict(
            zip(self.vehicle_ids, self.vehicle_names)
        )
        self.formation_type = str(
            self.get_parameter('formation_type').value
        )
        self.spacing = float(
            self.get_parameter('formation_spacing').value
        )
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        self.local_position_timeout = float(
            self.get_parameter('local_position_timeout').value
        )
        self.cpf_config = CpfConfig(
            enabled=bool(
                self.get_parameter('cpf_enabled').value
            ),
            attraction_gain=float(
                self.get_parameter('cpf_attraction_gain').value
            ),
            repulsion_gain=float(
                self.get_parameter('cpf_repulsion_gain').value
            ),
            safe_distance=float(
                self.get_parameter('cpf_safe_distance').value
            ),
            influence_distance=float(
                self.get_parameter('cpf_influence_distance').value
            ),
            max_speed=float(
                self.get_parameter('cpf_max_speed').value
            ),
            output_velocity=bool(
                self.get_parameter('cpf_output_velocity').value
            ),
        )

        self.status = None
        self.command = None
        self.last_command_ns = 0
        self.last_update_ns = (
            self.get_clock().now().nanoseconds
        )
        self.warned_inactive = False
        self.current_dt_seconds = 0.01
        self.neighbor_states = {}
        self.neighbor_subscriptions = []
        self.create_neighbor_subscriptions()

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

        frequency = float(
            self.get_parameter('control_frequency').value
        )
        if frequency <= 2.0:
            raise ValueError(
                'control_frequency must be greater than 2 Hz'
            )

        self.timer = self.create_timer(
            1.0 / frequency,
            self.control_loop,
        )

        self.get_logger().info(
            f'Distributed vehicle {self.vehicle_id} '
            f'controller ready at {frequency:.1f} Hz'
        )

    def create_neighbor_subscriptions(self):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for vehicle_id in self.vehicle_ids:
            if vehicle_id == self.vehicle_id:
                continue

            namespace = self.names_by_id[vehicle_id]
            state = VehicleState(
                vehicle_id=vehicle_id,
                namespace=namespace,
            )
            self.neighbor_states[vehicle_id] = state
            self.neighbor_subscriptions.append(
                self.create_subscription(
                    VehicleLocalPosition,
                    f'/{namespace}/fmu/out/vehicle_local_position_v1',
                    lambda message, bound_id=vehicle_id: (
                        self.neighbor_position_callback(
                            bound_id,
                            message,
                        )
                    ),
                    qos,
                )
            )

    def neighbor_position_callback(self, vehicle_id, message):
        state = self.neighbor_states[vehicle_id]
        state.position_received = True
        state.position_valid = bool(
            message.xy_valid
            and message.z_valid
        )
        state.last_position_update_ns = (
            self.get_clock().now().nanoseconds
        )

        if not state.position_valid:
            state.position_local_enu = None
            return

        position_ned = VectorNED(
            north=float(message.x),
            east=float(message.y),
            down=float(message.z),
        )
        state.position_local_enu = ned_to_enu(position_ned)

    def status_callback(self, message):
        self.status = message

    def command_callback(self, message):
        if self.status is None:
            return

        if int(message.leader_id) != int(self.status.leader_id):
            return

        if (
            int(message.leader_generation)
            != int(self.status.leader_generation)
        ):
            return

        self.command = message
        self.last_command_ns = (
            self.get_clock().now().nanoseconds
        )

    def control_loop(self):
        now_ns = self.get_clock().now().nanoseconds
        dt_seconds = max(
            0.0,
            min(0.1, (now_ns - self.last_update_ns) / 1e9),
        )
        self.current_dt_seconds = dt_seconds
        self.last_update_ns = now_ns

        if not self.active_for_formation(now_ns):
            return

        self.apply_command(dt_seconds)

        setpoint = self.calculate_setpoint()
        if setpoint is None:
            return

        self.vehicle.publish_offboard_heartbeat(
            use_velocity=(setpoint.velocity_local_enu is not None)
        )
        self.vehicle.publish_setpoint(setpoint)

    def active_for_formation(self, now_ns):
        if self.status is None:
            return False

        active = (
            self.status.control_mode == 'distributed'
            and self.status.mission_state == 'FORMATION'
        )
        if not active:
            return False

        if self.command is None:
            return False

        age = (now_ns - self.last_command_ns) / 1e9
        if age > self.command_timeout:
            if not self.warned_inactive:
                self.get_logger().warning(
                    'Formation command timeout; holding last setpoint'
                )
                self.warned_inactive = True
            return False

        self.warned_inactive = False
        return True

    def apply_command(self, dt_seconds):
        if self.command.command == FormationCommand.MOVE:
            velocity = VectorENU(
                east=float(self.command.velocity_east),
                north=float(self.command.velocity_north),
                up=float(self.command.velocity_up),
            )
            self.formation_reference = advance_reference(
                self.formation_reference,
                velocity,
                float(self.command.yaw_rate),
                dt_seconds,
            )
        elif self.command.command == FormationCommand.SET_YAW:
            self.formation_reference = set_reference_yaw(
                self.formation_reference,
                float(self.command.yaw_enu),
            )

    def collect_cpf_states(self):
        states = {
            self.vehicle_id: self.vehicle.get_state(),
        }
        now_ns = self.get_clock().now().nanoseconds

        for vehicle_id, state in self.neighbor_states.items():
            age = (
                now_ns - state.last_position_update_ns
            ) / 1e9
            if (
                state.position_valid
                and state.position_local_enu is not None
                and 0.0 <= age <= self.local_position_timeout
            ):
                states[vehicle_id] = state

        return states

    def calculate_setpoint(self):
        if self.vehicle_id not in self.vehicle_ids:
            self.get_logger().error(
                f'Vehicle {self.vehicle_id} missing from vehicle_ids'
            )
            return None

        offsets = get_shape(
            name=self.formation_type,
            spacing=self.spacing,
            vehicle_ids=self.vehicle_ids,
            leader_id=int(self.status.leader_id),
        )

        rotated_offset = rotate_offset(
            offsets[self.vehicle_id],
            self.formation_reference.yaw_enu,
        )
        target_world = add_vectors(
            self.formation_reference.position_world_enu,
            rotated_offset,
        )
        target_local = subtract_vectors(
            target_world,
            self.vehicle_origin,
        )

        nominal = VehicleSetpoint(
            position_local_enu=target_local,
            yaw_local_enu=self.formation_reference.yaw_enu,
        )

        safe_setpoints = apply_cpf_to_setpoints(
            states=self.collect_cpf_states(),
            nominal_setpoints={self.vehicle_id: nominal},
            vehicle_origins=self.vehicle_origins,
            config=self.cpf_config,
            dt_seconds=self.current_dt_seconds,
        )

        return safe_setpoints[self.vehicle_id]


def main(args=None):
    rclpy.init(args=args)
    node = DistributedVehicleNode()

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
