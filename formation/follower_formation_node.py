import itertools
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from formation.coordinate_convert import VectorENU
from formation.cpf_avoidance import CpfConfig, apply_cpf_to_setpoints
from formation.formation_controller import (
    add_vectors,
    rotate_offset,
    subtract_vectors,
    vector_distance,
)
from formation.msg import FormationStatus
from formation.mission_manager import MissionState
from formation.vehicle_interface import VehicleInterface, VehicleSetpoint


CONTROL_MODE_LEADER_FOLLOWER = 'leader_follower'


class FollowerFormationNode(Node):
    """Leader-relative follower formation controller.

    This node owns only follower heartbeat/setpoints during FORMATION.
    The leader is treated as the anchor source and is controlled by
    leader_control_node or another leader controller.
    """

    def __init__(self):
        super().__init__('follower_formation_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_frequency', 50.0),
                ('leader_id', 1),
                ('formation_type', 'line'),
                ('formation_spacing', 1.2),
                ('formation_reference_yaw_enu', 0.0),
                ('formation_position_tolerance', 0.5),
                ('formation_speed_tolerance', 0.15),
                ('formation_hold_duration', 2.0),
                ('local_position_timeout', 2.0),
                ('yaw_debug_enabled', False),
                ('yaw_debug_interval', 1.0),
                ('target_debug_enabled', False),
                ('target_debug_interval', 1.0),
                ('vehicle_names', ['MAV1', 'MAV2', 'MAV3']),
                ('vehicle_system_ids', [1, 2, 3]),
                ('vehicle_origins_enu', [
                    0.0, 0.0, 0.0,
                    2.0, 0.0, 0.0,
                    0.0, 2.0, 0.0,
                ]),
                ('slot_assignment_lock', True),
                ('cpf_enabled', True),
                ('cpf_attraction_gain', 0.5),
                ('cpf_repulsion_gain', 1.2),
                ('cpf_safe_distance', 0.8),
                ('cpf_influence_distance', 3.0),
                ('cpf_max_speed', 0.6),
                ('cpf_output_velocity', True),
                ('fence_enabled', False),
                ('fence_world_x_min', -50.0),
                ('fence_world_x_max', 50.0),
                ('fence_world_y_min', -50.0),
                ('fence_world_y_max', 50.0),
                ('fence_height_min', 0.0),
                ('fence_height_max', 20.0),
                ('fence_brake_distance_m', 0.35),
            ],
        )

        frequency = float(
            self.get_parameter('control_frequency').value
        )
        if frequency <= 2.0:
            raise ValueError(
                'control_frequency must be greater than 2 Hz'
            )
        self.control_period = 1.0 / frequency

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

        origin_values = list(
            self.get_parameter('vehicle_origins_enu').value
        )
        expected_origin_values = 3 * len(self.vehicles)
        if len(origin_values) != expected_origin_values:
            raise ValueError(
                'vehicle_origins_enu must contain exactly '
                f'{expected_origin_values} values'
            )

        self.vehicle_origins = {
            vehicle_id: VectorENU(
                east=float(origin_values[index]),
                north=float(origin_values[index + 1]),
                up=float(origin_values[index + 2]),
            )
            for index, vehicle_id in zip(
                range(0, len(origin_values), 3),
                sorted(self.vehicles),
            )
        }

        self.active_leader_id = int(
            self.get_parameter('leader_id').value
        )
        self.leader_generation = 0
        self.mission_state = MissionState.IDLE.name
        self.control_mode = ''
        self.formation_type = str(
            self.get_parameter('formation_type').value
        )
        self.spacing = float(
            self.get_parameter('formation_spacing').value
        )
        self.yaw_enu = float(
            self.get_parameter('formation_reference_yaw_enu').value
        )
        self.formation_position_tolerance = float(
            self.get_parameter(
                'formation_position_tolerance'
            ).value
        )
        self.formation_speed_tolerance = float(
            self.get_parameter(
                'formation_speed_tolerance'
            ).value
        )
        self.formation_hold_duration = float(
            self.get_parameter(
                'formation_hold_duration'
            ).value
        )
        self.slot_assignment_lock = bool(
            self.get_parameter('slot_assignment_lock').value
        )
        self.slot_offsets = None
        self.last_leader_generation = self.leader_generation

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

        self.maximum_position_error = 0.0
        self.maximum_follower_speed = math.inf
        self.last_target_debug_ns = 0
        self.formation_ready_since_ns = None
        self.last_formation_ready = False

        self.follower_status_publisher = self.create_publisher(
            FormationStatus,
            '/formation/follower_status',
            10,
        )

        self.status_subscription = self.create_subscription(
            FormationStatus,
            '/formation/status',
            self.status_callback,
            10,
        )
        self.timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )
        self.status_timer = self.create_timer(
            0.1,
            self.publish_follower_status,
        )

        self.get_logger().info(
            f'Follower formation node ready at {frequency:.1f} Hz'
        )

    def status_callback(self, message):
        leader_changed = (
            int(message.leader_id) != self.active_leader_id
            or int(message.leader_generation)
            != self.leader_generation
        )

        previous_formation_type = self.formation_type
        previous_spacing = self.spacing

        self.active_leader_id = int(message.leader_id)
        self.leader_generation = int(message.leader_generation)
        self.mission_state = str(message.mission_state)
        self.control_mode = str(message.control_mode)
        self.formation_type = str(message.formation_type)
        if getattr(message, 'spacing', 0.0) > 0.0:
            self.spacing = float(message.spacing)

        formation_changed = (
            self.formation_type != previous_formation_type
            or abs(self.spacing - previous_spacing) > 1e-6
        )

        if leader_changed or formation_changed:
            self.slot_offsets = None
            self.last_leader_generation = self.leader_generation
            self.formation_ready_since_ns = None
            self.last_formation_ready = False

        if formation_changed:
            self.get_logger().info(
                'Formation shape updated to '
                f'{self.formation_type}, spacing '
                f'{self.spacing:.2f} m; recalculating slots'
            )

    def formation_active(self):
        return bool(
            self.mission_state == MissionState.FORMATION.name
            and self.control_mode == CONTROL_MODE_LEADER_FOLLOWER
        )

    def get_states(self):
        return {
            vehicle_id: vehicle.get_state()
            for vehicle_id, vehicle in self.vehicles.items()
        }

    def valid_states(self, states):
        for state in states.values():
            if not state.position_valid:
                return False
            if state.position_local_enu is None:
                return False
        return True

    def world_positions(self, states):
        return {
            vehicle_id: add_vectors(
                self.vehicle_origins[vehicle_id],
                state.position_local_enu,
            )
            for vehicle_id, state in states.items()
        }

    def shape_slot_offsets(self):
        spacing = float(self.spacing)

        if self.formation_type == 'line':
            return [
                VectorENU(0.0, spacing, 0.0),
                VectorENU(0.0, 0.0, 0.0),
                VectorENU(0.0, -spacing, 0.0),
            ]

        if self.formation_type == 'triangle':
            return [
                VectorENU(0.0, 0.0, 0.0),
                VectorENU(-spacing, spacing, 0.0),
                VectorENU(-spacing, -spacing, 0.0),
            ]

        if self.formation_type == 'v_shape':
            return [
                VectorENU(0.0, 0.0, 0.0),
                VectorENU(-spacing, spacing, 0.0),
                VectorENU(-spacing, -spacing, 0.0),
            ]

        if self.formation_type == 'column':
            return [
                VectorENU(spacing, 0.0, 0.0),
                VectorENU(0.0, 0.0, 0.0),
                VectorENU(-spacing, 0.0, 0.0),
            ]

        raise ValueError(f'Unknown formation: {self.formation_type}')

    def leader_slot_index(self, slot_offsets):
        if self.formation_type in {'line', 'column'}:
            return min(1, len(slot_offsets) - 1)

        return 0

    def assign_slots_by_minimum_distance(self, states):
        positions = self.world_positions(states)
        vehicle_ids = sorted(positions)
        slot_offsets = self.shape_slot_offsets()

        if len(vehicle_ids) > len(slot_offsets):
            raise ValueError(
                f'Formation {self.formation_type} supports at most '
                f'{len(slot_offsets)} vehicles'
            )

        if self.active_leader_id not in positions:
            raise ValueError(
                f'Leader {self.active_leader_id} position unavailable'
            )

        leader_slot = self.leader_slot_index(slot_offsets)
        leader_offset = rotate_offset(
            slot_offsets[leader_slot],
            self.yaw_enu,
        )
        anchor = subtract_vectors(
            positions[self.active_leader_id],
            leader_offset,
        )

        usable_slot_indexes = list(range(len(vehicle_ids)))
        remaining_vehicle_ids = [
            vehicle_id
            for vehicle_id in vehicle_ids
            if vehicle_id != self.active_leader_id
        ]
        remaining_slot_indexes = [
            slot_index
            for slot_index in usable_slot_indexes
            if slot_index != leader_slot
        ]

        slot_world_positions = {
            slot_index: add_vectors(
                anchor,
                rotate_offset(slot_offsets[slot_index], self.yaw_enu),
            )
            for slot_index in usable_slot_indexes
        }

        best_cost = math.inf
        best_assignment = {
            self.active_leader_id: slot_offsets[leader_slot],
        }

        for permutation in itertools.permutations(
            remaining_slot_indexes,
            len(remaining_vehicle_ids),
        ):
            cost = 0.0
            for vehicle_id, slot_index in zip(
                remaining_vehicle_ids,
                permutation,
            ):
                cost += vector_distance(
                    positions[vehicle_id],
                    slot_world_positions[slot_index],
                )

            if cost < best_cost:
                best_cost = cost
                best_assignment = {
                    self.active_leader_id: slot_offsets[leader_slot],
                }
                for vehicle_id, slot_index in zip(
                    remaining_vehicle_ids,
                    permutation,
                ):
                    best_assignment[vehicle_id] = (
                        slot_offsets[slot_index]
                    )

        self.slot_offsets = best_assignment
        self.get_logger().info(
            'Slot assignment locked relative to latest leader position: '
            + ', '.join(
                f'MAV{vehicle_id}=({offset.east:.2f}, '
                f'{offset.north:.2f}, {offset.up:.2f})'
                for vehicle_id, offset
                in sorted(self.slot_offsets.items())
            )
        )

    def lock_current_offsets(self, states):
        positions = self.world_positions(states)

        if self.active_leader_id not in positions:
            raise ValueError(
                f'Leader {self.active_leader_id} position unavailable'
            )

        leader_world = positions[self.active_leader_id]
        self.slot_offsets = {
            self.active_leader_id: VectorENU(0.0, 0.0, 0.0),
        }

        for vehicle_id, position in positions.items():
            if vehicle_id == self.active_leader_id:
                continue

            self.slot_offsets[vehicle_id] = subtract_vectors(
                position,
                leader_world,
            )

        self.get_logger().info(
            'Current relative offsets locked for formation stabilize: '
            + ', '.join(
                f'MAV{vehicle_id}=({offset.east:.2f}, '
                f'{offset.north:.2f}, {offset.up:.2f})'
                for vehicle_id, offset
                in sorted(self.slot_offsets.items())
            )
        )

    def calculate_follower_setpoints(self, states):
        if self.slot_offsets is None:
            self.lock_current_offsets(states)

        if self.active_leader_id not in states:
            return {}
        if self.active_leader_id not in self.slot_offsets:
            return {}

        leader_state = states[self.active_leader_id]
        leader_world = add_vectors(
            self.vehicle_origins[self.active_leader_id],
            leader_state.position_local_enu,
        )
        leader_offset = self.slot_offsets[self.active_leader_id]
        anchor_world = subtract_vectors(
            leader_world,
            leader_offset,
        )

        setpoints = {}
        target_debug_rows = []
        maximum_error = 0.0

        for vehicle_id, state in states.items():
            if vehicle_id == self.active_leader_id:
                continue
            if vehicle_id not in self.slot_offsets:
                continue

            rotated_offset = self.slot_offsets[vehicle_id]
            target_world = add_vectors(
                anchor_world,
                rotated_offset,
            )
            target_local = subtract_vectors(
                target_world,
                self.vehicle_origins[vehicle_id],
            )
            setpoints[vehicle_id] = VehicleSetpoint(
                position_local_enu=target_local,
                yaw_local_enu=state.yaw_local_enu,
            )

            current_world = add_vectors(
                self.vehicle_origins[vehicle_id],
                state.position_local_enu,
            )
            target_debug_rows.append({
                'vehicle_id': vehicle_id,
                'leader_world': leader_world,
                'anchor_world': anchor_world,
                'offset': self.slot_offsets[vehicle_id],
                'rotated_offset': rotated_offset,
                'current_world': current_world,
                'target_world': target_world,
                'target_local': target_local,
            })
            maximum_error = max(
                maximum_error,
                vector_distance(current_world, target_world),
            )

        self.maximum_position_error = maximum_error
        safe_setpoints = apply_cpf_to_setpoints(
            states=states,
            nominal_setpoints=setpoints,
            vehicle_origins=self.vehicle_origins,
            config=self.cpf_config,
            dt_seconds=self.control_period,
        )
        self.log_follower_target_debug_if_enabled(
            target_debug_rows,
            safe_setpoints,
        )
        return safe_setpoints

    def log_follower_target_debug_if_enabled(
        self,
        rows,
        safe_setpoints,
    ):
        enabled = bool(
            self.get_parameter('target_debug_enabled').value
        )
        if not enabled:
            return

        interval = float(
            self.get_parameter('target_debug_interval').value
        )
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_target_debug_ns < interval * 1e9:
            return

        self.last_target_debug_ns = now_ns

        for row in rows:
            vehicle_id = row['vehicle_id']
            setpoint = safe_setpoints.get(vehicle_id)
            velocity_text = 'none'
            if (
                setpoint is not None
                and setpoint.velocity_local_enu is not None
            ):
                velocity = setpoint.velocity_local_enu
                velocity_text = (
                    f'({velocity.east:.2f}, '
                    f'{velocity.north:.2f}, {velocity.up:.2f})'
                )

            leader = row['leader_world']
            offset = row['offset']
            rotated = row['rotated_offset']
            current = row['current_world']
            target_world = row['target_world']
            target_local = row['target_local']

            self.get_logger().info(
                f'Follower target debug MAV{vehicle_id}: '
                f'leader_world_enu=({leader.east:.2f}, '
                f'{leader.north:.2f}, {leader.up:.2f}), '
                f'slot_offset=({offset.east:.2f}, '
                f'{offset.north:.2f}, {offset.up:.2f}), '
                f'rotated_offset=({rotated.east:.2f}, '
                f'{rotated.north:.2f}, {rotated.up:.2f}), '
                f'current_world_enu=({current.east:.2f}, '
                f'{current.north:.2f}, {current.up:.2f}), '
                f'target_world_enu=({target_world.east:.2f}, '
                f'{target_world.north:.2f}, {target_world.up:.2f}), '
                f'target_local_enu=({target_local.east:.2f}, '
                f'{target_local.north:.2f}, {target_local.up:.2f}), '
                f'cpf_velocity_enu={velocity_text}'
            )

    def control_loop(self):
        if not self.formation_active():
            return

        states = self.get_states()
        if not self.valid_states(states):
            return

        setpoints = self.calculate_follower_setpoints(states)

        for vehicle_id, setpoint in setpoints.items():
            vehicle = self.vehicles[vehicle_id]
            use_velocity = bool(
                setpoint.velocity_local_enu is not None
            )
            vehicle.publish_offboard_heartbeat(
                use_velocity=use_velocity
            )
            vehicle.publish_setpoint(setpoint)

    def vector_speed(self, velocity):
        if velocity is None:
            return math.inf

        return math.sqrt(
            velocity.east ** 2
            + velocity.north ** 2
            + velocity.up ** 2
        )

    def maximum_active_follower_speed(self):
        states = self.get_states()
        speeds = [
            self.vector_speed(state.velocity_local_enu)
            for vehicle_id, state in states.items()
            if vehicle_id != self.active_leader_id
        ]

        if not speeds:
            return math.inf

        return max(speeds)

    def update_formation_ready_state(self):
        now_ns = self.get_clock().now().nanoseconds
        self.maximum_follower_speed = (
            self.maximum_active_follower_speed()
        )

        stable_now = bool(
            self.formation_active()
            and self.slot_offsets is not None
            and self.maximum_position_error
            <= self.formation_position_tolerance
            and self.maximum_follower_speed
            <= self.formation_speed_tolerance
        )

        if not stable_now:
            self.formation_ready_since_ns = None
            return False

        if self.formation_ready_since_ns is None:
            self.formation_ready_since_ns = now_ns
            return False

        held_seconds = (
            now_ns - self.formation_ready_since_ns
        ) / 1e9

        return held_seconds >= self.formation_hold_duration

    def publish_follower_status(self):
        ready = self.update_formation_ready_state()

        message = FormationStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.leader_id = int(self.active_leader_id)
        message.mission_state = self.mission_state
        message.control_mode = self.control_mode
        message.formation_type = self.formation_type
        message.spacing = float(self.spacing)
        message.formation_ready = ready
        message.maximum_position_error = float(
            self.maximum_position_error
        )
        message.leader_healthy = True
        message.leader_fault_reason = ''
        message.leader_generation = int(self.leader_generation)
        message.failed_leader_id = 0
        message.awaiting_ground_confirmation = False

        self.follower_status_publisher.publish(message)

        if ready and not self.last_formation_ready:
            self.get_logger().info(
                'FORMATION COMPLETE: followers held position error '
                f'<= {self.formation_position_tolerance:.2f} m and '
                f'speed <= {self.formation_speed_tolerance:.2f} m/s '
                f'for {self.formation_hold_duration:.1f} s'
            )
        self.last_formation_ready = ready



def main(args=None):
    rclpy.init(args=args)
    node = FollowerFormationNode()

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
