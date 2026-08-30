import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

from formation.msg import FormationCommand, FormationStatus
from formation.coordinate_convert import VectorENU
from formation.cpf_avoidance import (
    CpfConfig,
    add_vectors as add_cpf_vectors,
    limit_vector,
    repulsion_from_neighbors,
    scale_vector,
)
from formation.fence_limiter import (
    FenceConfig,
    limit_velocity_near_fence,
)
from formation.srv import SetLeader
from formation.formation_controller import (
    CentralizedFormationController,
    DistributedFormationController,
    FormationReference,
    add_vectors,
    subtract_vectors,
    vector_distance,
)
from formation.leader_manager import LeaderManager
from formation.mission_manager import MissionManager, MissionState
from formation.safety_manager import SafetyManager
from formation.vehicle_interface import VehicleInterface, VehicleSetpoint

"""
Mission-layer node for the formation package.

Current responsibility:
    mission_node
        Owns mission state, synchronized takeoff, synchronized landing,
        emergency landing, leader selection, safety checks and status.

Next split names:
    leader_control_node
        Future node that owns only the leader setpoint during FORMATION.

    follower_control_node
        Future node that owns only follower setpoints during FORMATION.

    safety_monitor_node
        Future optional node that monitors all UAVs and can request
        emergency landing.

Naming rule:
    "leader" means command/anchor source.
    "slot" means formation position such as left, center or right.
    A leader does not have to be the center slot.
"""


MISSION_NODE_NAME = 'mission_node'

CONTROL_MODE_CENTRALIZED = 'centralized'
CONTROL_MODE_DISTRIBUTED = 'distributed'
CONTROL_MODE_LEADER_FOLLOWER = 'leader_follower'


class MissionNode(Node):
    """Mission manager node.

    The executable is still named ``formation_node`` for compatibility,
    but this class is intentionally named MissionNode because this file
    is becoming the mission layer before leader/follower control is
    split into separate nodes.
    """

    def __init__(self):
        super().__init__(MISSION_NODE_NAME)

        # 讀取yaml參數
        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_frequency', 100.0),
                ('control_mode', 'centralized'),
                ('formation_type', 'triangle'),
                ('formation_spacing', 2.0),
                ('leader_id', 1),
                ('takeoff_height', 0.5),
                ('takeoff_climb_rate', 0.25),
                ('hover_stabilize_duration', 1.0),
                ('leader_stabilize_duration', 1.0),
                ('liftoff_after_arm_timeout', 10.0),
                ('ready_hold_duration', 2.0),
                ('offboard_warmup_duration', 2.0),
                ('local_position_timeout', 2.0),
                ('takeoff_altitude_tolerance', 0.3),
                ('leader_position_timeout', 2.0),
                ('leader_failure_hold_duration', 1.0),
                ('land_command_interval', 1.0),
                ('vehicle_origins_enu', [
                    0.0, 0.0, 0.0,
                    2.0, 0.0, 0.0,
                    0.0, 2.0, 0.0,
                ]),
                ('minimum_vehicle_distance', 1.0),
                ('maximum_altitude', 20.0),
                ('maximum_speed', 8.0),
                ('geofence_radius', 50.0),
                ('maximum_setpoint_jump', 5.0),
                ('fence_enabled', False),
                ('fence_world_x_min', -50.0),
                ('fence_world_x_max', 50.0),
                ('fence_world_y_min', -50.0),
                ('fence_world_y_max', 50.0),
                ('fence_height_min', 0.0),
                ('fence_height_max', 20.0),
                ('fence_brake_distance_m', 0.35),
                ('near_fence_margin_m', 0.3),
                ('formation_position_tolerance', 0.5),
                ('formation_speed_tolerance', 0.15),
                ('formation_hold_duration', 2.0),
                ('formation_footprint_margin', 0.1),
                ('tracking_kp', 0.8),
                ('tracking_max_speed', 0.5),
                ('tracking_deadband', 0.03),
                ('formation_reference_enu', [
                    0.0, 0.0, 3.0,
                ]),
                ('formation_reference_yaw_enu', 0.0),
                ('command_timeout', 1.0),
                ('arm_retry_interval', 0.5),
                ('arm_retry_timeout', 8.0),
                ('takeoff_diagnostic_interval', 2.0),
                ('yaw_debug_enabled', False),
                ('yaw_debug_interval', 1.0),
                ('cpf_enabled', False),
                ('cpf_attraction_gain', 0.8),
                ('cpf_repulsion_gain', 1.2),
                ('cpf_safe_distance', 1.5),
                ('cpf_influence_distance', 4.0),
                ('cpf_max_speed', 0.8),
                ('cpf_output_velocity', True),
            ],
        )

        #建立無人機介面
        self.vehicles = {
            1: VehicleInterface(self, 'MAV1', 1),
            2: VehicleInterface(self, 'MAV2', 2),
            3: VehicleInterface(self, 'MAV3', 3),
        }

        # 建立任務管理器
        # IDLE -> WAITING_READY -> OFFBOARD_WARMUP -> TAKEOFF -> FORMATION -> LANDING/LANDED
        self.mission_manager = MissionManager(
            takeoff_height=self.get_parameter(
                'takeoff_height'
            ).value,
            ready_hold_duration=self.get_parameter(
                'ready_hold_duration'
            ).value,
            warmup_duration=self.get_parameter(
                'offboard_warmup_duration'
            ).value,
            position_timeout=self.get_parameter(
                'local_position_timeout'
            ).value,
            altitude_tolerance=self.get_parameter(
                'takeoff_altitude_tolerance'
            ).value,
            land_command_interval=self.get_parameter(
                'land_command_interval'
            ).value,
            liftoff_after_arm_timeout=self.get_parameter(
                'liftoff_after_arm_timeout'
            ).value,
            takeoff_climb_rate=self.get_parameter(
                'takeoff_climb_rate'
            ).value,
            hover_stabilize_duration=self.get_parameter(
                'hover_stabilize_duration'
            ).value,
            leader_stabilize_duration=self.get_parameter(
                'leader_stabilize_duration'
            ).value,
        )

        # 建立領導者管理器
        leader_id = self.get_parameter('leader_id').value
        self.leader_manager = LeaderManager(
            initial_leader_id=leader_id,
            position_timeout=self.get_parameter(
                'leader_position_timeout'
            ).value,
            failure_hold_duration=self.get_parameter(
                'leader_failure_hold_duration'
            ).value,
        )

        # 讀取無人機原始位子
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

        # 建立安全管理器
        self.safety_manager = SafetyManager(
            vehicle_origins=self.vehicle_origins,
            position_timeout=self.get_parameter(
                'local_position_timeout'
            ).value,
            minimum_distance=self.get_parameter(
                'minimum_vehicle_distance'
            ).value,
            maximum_altitude=self.get_parameter(
                'maximum_altitude'
            ).value,
            maximum_speed=self.get_parameter(
                'maximum_speed'
            ).value,
            geofence_radius=self.get_parameter(
                'geofence_radius'
            ).value,
            maximum_setpoint_jump=self.get_parameter(
                'maximum_setpoint_jump'
            ).value,
            fence_enabled=self.get_parameter(
                'fence_enabled'
            ).value,
            fence_world_x_min=self.get_parameter(
                'fence_world_x_min'
            ).value,
            fence_world_x_max=self.get_parameter(
                'fence_world_x_max'
            ).value,
            fence_world_y_min=self.get_parameter(
                'fence_world_y_min'
            ).value,
            fence_world_y_max=self.get_parameter(
                'fence_world_y_max'
            ).value,
            fence_height_min=self.get_parameter(
                'fence_height_min'
            ).value,
            fence_height_max=self.get_parameter(
                'fence_height_max'
            ).value,
            near_fence_margin_m=self.get_parameter(
                'near_fence_margin_m'
            ).value,
        )

        # 讀取yaml設定
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
        self.formation_footprint_margin = float(
            self.get_parameter(
                'formation_footprint_margin'
            ).value
        )
        self.tracking_kp = float(
            self.get_parameter('tracking_kp').value
        )
        self.tracking_max_speed = float(
            self.get_parameter('tracking_max_speed').value
        )
        self.tracking_deadband = float(
            self.get_parameter('tracking_deadband').value
        )

        mode = self.get_parameter('control_mode').value
        self.control_mode = str(mode)
        self.formation_type = str(
            self.get_parameter('formation_type').value
        )
        self.formation_spacing = float(
            self.get_parameter('formation_spacing').value
        )
        frequency = float(
            self.get_parameter('control_frequency').value
        )
        if frequency <= 2.0:
            raise ValueError(
                'control_frequency must be greater than 2 Hz'
            )

        self.control_period = 1.0 / frequency


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

        if mode == CONTROL_MODE_CENTRALIZED:
            reference_values = list(
                self.get_parameter(
                    'formation_reference_enu'
                ).value
            )
            if len(reference_values) != 3:
                raise ValueError(
                    'formation_reference_enu must contain '
                    'east, north and up'
                )

            formation_reference = FormationReference(
                position_world_enu=VectorENU(
                    east=float(reference_values[0]),
                    north=float(reference_values[1]),
                    up=float(reference_values[2]),
                ),
                yaw_enu=float(
                    self.get_parameter(
                        'formation_reference_yaw_enu'
                    ).value
                ),
            )

            self.controller = CentralizedFormationController(
                formation_type=self.formation_type,
                spacing=self.formation_spacing,
                vehicle_origins=self.vehicle_origins,
                formation_reference=formation_reference,
                control_period=self.control_period,
                cpf_config=self.cpf_config,
            )
        elif mode in {
            CONTROL_MODE_DISTRIBUTED,
            CONTROL_MODE_LEADER_FOLLOWER,
        }:
            # In distributed / leader_follower modes, FORMATION
            # setpoints are produced by external control nodes.
            self.controller = DistributedFormationController()
        else:
            raise ValueError(f'Unknown control mode: {mode}')

        # 設定指令接收與超時管理
        self.latest_command = None
        self.last_command_receive_ns = 0
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        self.command_timeout_reported = False
        self.arm_retry_interval = float(
            self.get_parameter('arm_retry_interval').value
        )
        self.arm_retry_timeout = float(
            self.get_parameter('arm_retry_timeout').value
        )
        self.takeoff_diagnostic_interval = float(
            self.get_parameter(
                'takeoff_diagnostic_interval'
            ).value
        )
        self.last_takeoff_diagnostic_ns = 0
        self.arm_retry_start_ns = 0
        self.last_arm_retry_ns = 0
        self.arm_retry_timeout_reported = False

        self.timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        # 建立server 包含起飛 降落 指定leader 隊伍形狀
        self.takeoff_service = self.create_service(
            Trigger,
            '/formation/takeoff',
            self.takeoff_callback,
        )
        self.land_service = self.create_service(
            Trigger,
            '/formation/land',
            self.land_callback,
        )
        self.confirm_fault_service = self.create_service(
            Trigger,
            '/formation/confirm_fault',
            self.confirm_fault_callback,
        )
        self.set_leader_service = self.create_service(
            SetLeader,
            '/formation/set_leader',
            self.set_leader_callback,
        )

        # 持續接收新指令 整隊現在的高階控制意圖是什麼
        self.command_subscription = self.create_subscription(
            FormationCommand,
            '/formation/command',
            self.command_callback,
            10,
        )

        self.follower_control_ready = False
        self.follower_maximum_position_error = 0.0
        self.formation_complete_reported = False
        self.centralized_offsets = None
        self.centralized_active_move_sequence = None
        self.centralized_active_velocity = VectorENU(0.0, 0.0, 0.0)
        self.centralized_maximum_position_error = 0.0
        self.centralized_maximum_speed = 0.0
        self.centralized_formation_ready_since_ns = None

        self.follower_status_subscription = (
            self.create_subscription(
                FormationStatus,
                '/formation/follower_status',
                self.follower_status_callback,
                10,
            )
        )

        # 發布狀態
        self.status_publisher = self.create_publisher(
            FormationStatus,
            '/formation/status',
            10,
        )
        self.status_timer = self.create_timer(
            0.1,
            self.publish_formation_status,
        )

        self.last_mission_state = self.mission_manager.state
        self.last_leader_warning_reason = ''

        self.get_logger().info(
            f'Formation controller ready at {frequency:.1f} Hz'
        )

    def command_sequence(self, command):
        return int(getattr(command, 'sequence', 0))

    def central_world_positions(self, states):
        positions = {}
        for vehicle_id, state in states.items():
            if vehicle_id not in self.vehicle_origins:
                continue
            if not state.position_valid:
                continue
            if state.position_local_enu is None:
                continue

            positions[vehicle_id] = add_vectors(
                self.vehicle_origins[vehicle_id],
                state.position_local_enu,
            )

        return positions

    def capture_centralized_snapshot(self, states):
        positions = self.central_world_positions(states)
        leader_id = self.leader_manager.leader_id

        if leader_id not in positions:
            self.get_logger().warning(
                'Cannot snapshot formation: leader position unavailable'
            )
            return False

        missing = set(self.vehicles) - set(positions)
        if missing:
            self.get_logger().warning(
                'Cannot snapshot formation: missing positions for '
                + ', '.join(f'MAV{vehicle_id}' for vehicle_id in sorted(missing))
            )
            return False

        leader_position = positions[leader_id]
        self.centralized_offsets = {
            vehicle_id: subtract_vectors(position, leader_position)
            for vehicle_id, position in positions.items()
        }
        self.centralized_formation_ready_since_ns = None
        self.formation_complete_reported = False

        self.get_logger().info(
            'SNAPSHOT_FORMATION locked offsets: '
            + ', '.join(
                f'MAV{vehicle_id}=({offset.east:.2f}, '
                f'{offset.north:.2f}, {offset.up:.2f})'
                for vehicle_id, offset
                in sorted(self.centralized_offsets.items())
            )
        )
        return True

    def predicted_leader_target(self, leader_position, command):
        return VectorENU(
            east=(
                leader_position.east
                + float(command.velocity_east) * float(command.duration)
            ),
            north=(
                leader_position.north
                + float(command.velocity_north) * float(command.duration)
            ),
            up=(
                leader_position.up
                + float(command.velocity_up) * float(command.duration)
            ),
        )

    def position_inside_fence_with_margin(self, position):
        if not self.fence_config.enabled:
            return True, ''

        margin = self.formation_footprint_margin
        checks = [
            (
                position.east >= self.fence_config.world_x_min + margin,
                f'east {position.east:.2f} < '
                f'x_min+margin {self.fence_config.world_x_min + margin:.2f}',
            ),
            (
                position.east <= self.fence_config.world_x_max - margin,
                f'east {position.east:.2f} > '
                f'x_max-margin {self.fence_config.world_x_max - margin:.2f}',
            ),
            (
                position.north >= self.fence_config.world_y_min + margin,
                f'north {position.north:.2f} < '
                f'y_min+margin {self.fence_config.world_y_min + margin:.2f}',
            ),
            (
                position.north <= self.fence_config.world_y_max - margin,
                f'north {position.north:.2f} > '
                f'y_max-margin {self.fence_config.world_y_max - margin:.2f}',
            ),
            (
                position.up >= self.fence_config.height_min + margin,
                f'up {position.up:.2f} < '
                f'z_min+margin {self.fence_config.height_min + margin:.2f}',
            ),
            (
                position.up <= self.fence_config.height_max - margin,
                f'up {position.up:.2f} > '
                f'z_max-margin {self.fence_config.height_max - margin:.2f}',
            ),
        ]

        for ok, reason in checks:
            if not ok:
                return False, reason

        return True, ''

    def validate_centralized_move_command(self, states, command):
        if self.centralized_offsets is None:
            if not self.capture_centralized_snapshot(states):
                return False

        positions = self.central_world_positions(states)
        leader_id = self.leader_manager.leader_id
        if leader_id not in positions:
            self.get_logger().error(
                'Rejected leader MOVE: leader position unavailable'
            )
            return False

        leader_target = self.predicted_leader_target(
            positions[leader_id],
            command,
        )

        debug_rows = []
        for vehicle_id, offset in sorted(self.centralized_offsets.items()):
            target = add_vectors(leader_target, offset)
            inside, reason = self.position_inside_fence_with_margin(
                target
            )
            debug_rows.append(
                f'MAV{vehicle_id}=({target.east:.2f}, '
                f'{target.north:.2f}, {target.up:.2f})'
            )
            if not inside:
                self.get_logger().error(
                    'Rejected leader MOVE: predicted MAV'
                    f'{vehicle_id} target ({target.east:.2f}, '
                    f'{target.north:.2f}, {target.up:.2f}) '
                    f'outside fence margin: {reason}'
                )
                return False

        self.get_logger().info(
            'Accepted leader MOVE footprint: leader_target='
            f'({leader_target.east:.2f}, {leader_target.north:.2f}, '
            f'{leader_target.up:.2f}), predicted_targets: '
            + ', '.join(debug_rows)
        )
        return True

    def active_centralized_command(self, now_ns):
        if self.latest_command is None:
            return None

        age_seconds = (now_ns - self.last_command_receive_ns) / 1e9
        if age_seconds > self.command_timeout:
            return None

        return self.latest_command

    def central_tracking_velocity(self, current_world, target_world):
        error = subtract_vectors(target_world, current_world)
        horizontal_error = (error.east ** 2 + error.north ** 2) ** 0.5
        vertical_error = abs(error.up)

        if (
            horizontal_error <= self.tracking_deadband
            and vertical_error <= self.tracking_deadband
        ):
            return VectorENU(0.0, 0.0, 0.0)

        return limit_vector(
            scale_vector(error, self.tracking_kp),
            self.tracking_max_speed,
        )

    def central_cpf_velocity(self, vehicle_id, current_world_positions):
        if not self.cpf_config.enabled:
            return VectorENU(0.0, 0.0, 0.0)

        return limit_vector(
            repulsion_from_neighbors(
                vehicle_id,
                current_world_positions,
                self.cpf_config,
            ),
            self.cpf_config.max_speed,
        )

    def vector_speed(self, vector):
        return (
            vector.east ** 2
            + vector.north ** 2
            + vector.up ** 2
        ) ** 0.5

    def make_velocity_setpoint(self, state, velocity):
        return VehicleSetpoint(
            position_local_enu=state.position_local_enu,
            yaw_local_enu=state.yaw_local_enu,
            velocity_local_enu=velocity,
        )

    def centralized_staged_setpoints(self, states, now_ns):
        mission_state = self.mission_manager.state
        leader_id = self.leader_manager.leader_id

        if mission_state not in {
            MissionState.LEADER_REPOSITION,
            MissionState.FOLLOWERS_APPROACH,
            MissionState.FORMATION_HOLD,
        }:
            return {}

        if self.centralized_offsets is None:
            if not self.capture_centralized_snapshot(states):
                return {}

        positions = self.central_world_positions(states)
        if leader_id not in positions:
            return {}

        setpoints = {}
        maximum_error = 0.0
        maximum_speed = 0.0
        zero = VectorENU(0.0, 0.0, 0.0)

        command = self.active_centralized_command(now_ns)
        leader_velocity = zero

        if (
            mission_state == MissionState.LEADER_REPOSITION
            and command is not None
            and command.command == FormationCommand.MOVE
        ):
            leader_velocity = VectorENU(
                east=float(command.velocity_east),
                north=float(command.velocity_north),
                up=float(command.velocity_up),
            )
            leader_velocity = limit_vector(
                leader_velocity,
                self.tracking_max_speed,
            )
            leader_velocity = limit_velocity_near_fence(
                positions[leader_id],
                leader_velocity,
                self.fence_config,
            )

        for vehicle_id, state in states.items():
            if not state.position_valid or state.position_local_enu is None:
                continue

            if vehicle_id == leader_id:
                velocity = leader_velocity
            elif mission_state == MissionState.LEADER_REPOSITION:
                velocity = zero
            else:
                target_world = add_vectors(
                    positions[leader_id],
                    self.centralized_offsets[vehicle_id],
                )
                current_world = positions.get(vehicle_id)
                if current_world is None:
                    continue

                tracking_velocity = self.central_tracking_velocity(
                    current_world,
                    target_world,
                )
                avoidance_velocity = self.central_cpf_velocity(
                    vehicle_id,
                    positions,
                )
                velocity = limit_vector(
                    add_cpf_vectors(
                        tracking_velocity,
                        avoidance_velocity,
                    ),
                    self.cpf_config.max_speed,
                )
                velocity = limit_velocity_near_fence(
                    current_world,
                    velocity,
                    self.fence_config,
                )
                maximum_error = max(
                    maximum_error,
                    vector_distance(current_world, target_world),
                )

            maximum_speed = max(maximum_speed, self.vector_speed(velocity))
            setpoints[vehicle_id] = self.make_velocity_setpoint(
                state,
                velocity,
            )

        self.centralized_maximum_position_error = maximum_error
        self.centralized_maximum_speed = maximum_speed
        self.update_centralized_formation_hold(now_ns)
        return setpoints

    def update_centralized_formation_hold(self, now_ns):
        if self.mission_manager.state not in {
            MissionState.FOLLOWERS_APPROACH,
            MissionState.FORMATION_HOLD,
        }:
            self.centralized_formation_ready_since_ns = None
            return

        stable_now = bool(
            self.centralized_maximum_position_error
            <= self.formation_position_tolerance
            and self.centralized_maximum_speed
            <= self.formation_speed_tolerance
        )

        if not stable_now:
            self.centralized_formation_ready_since_ns = None
            return

        if self.centralized_formation_ready_since_ns is None:
            self.centralized_formation_ready_since_ns = now_ns
            return

        held_seconds = (
            now_ns - self.centralized_formation_ready_since_ns
        ) / 1e9

        if held_seconds < self.formation_hold_duration:
            return

        if self.mission_manager.request_formation_hold():
            self.get_logger().info(
                'FORMATION COMPLETE: centralized formation is stable; '
                'landing command may be sent'
            )
            self.formation_complete_reported = True

    def control_loop(self):
        states = {
            vehicle_id: vehicle.get_state()
            for vehicle_id, vehicle in self.vehicles.items()
        }

        now_ns = self.get_clock().now().nanoseconds

        leader_check = None

        if self.mission_manager.state == MissionState.SNAPSHOT_FORMATION:
            if self.capture_centralized_snapshot(states):
                self.mission_manager.complete_snapshot()

        if self.mission_manager.state in {
            MissionState.WAITING_LEADER_COMMAND,
            MissionState.LEADER_REPOSITION,
            MissionState.FOLLOWERS_APPROACH,
            MissionState.FORMATION_HOLD,
            MissionState.FORMATION,
        }:
            self.process_formation_command(now_ns)

        if (
            self.control_mode == CONTROL_MODE_CENTRALIZED
            and self.mission_manager.leader_reposition_done(now_ns)
        ):
            if self.mission_manager.request_followers_approach():
                self.get_logger().info(
                    'Leader reposition complete; followers approach started'
                )

        if self.mission_manager.state in {
            MissionState.LEADER_REPOSITION,
            MissionState.FOLLOWERS_APPROACH,
            MissionState.FORMATION_HOLD,
            MissionState.FORMATION,
        }:
            flight_state_failure = (
                self.required_flight_state_failure(states)
            )
            if flight_state_failure:
                reason = (
                    'Required flight state failure: '
                    f'{flight_state_failure}'
                )
                if self.mission_manager.request_fault_landing(
                    reason
                ):
                    self.get_logger().error(
                        reason + '; landing all UAVs'
                    )

            leader_check = self.leader_manager.check_leader(
                states=states,
                now_ns=now_ns,
                require_armed=True,
                require_offboard=True,
                require_preflight=False,
            )

            if leader_check.healthy:
                self.last_leader_warning_reason = ''
            elif leader_check.failure_confirmed:
                reason = (
                    f'Leader {self.leader_manager.leader_id} '
                    f'failure: {leader_check.reason}'
                )
                if self.mission_manager.request_fault_landing(
                    reason
                ):
                    self.get_logger().error(
                        reason + '; landing all UAVs'
                    )
            elif (
                leader_check.reason
                != self.last_leader_warning_reason
            ):
                self.get_logger().warning(
                    'Leader health warning: '
                    f'{leader_check.reason}'
                )
                self.last_leader_warning_reason = (
                    leader_check.reason
                )

        should_check_safety = (
            self.mission_manager.state in {
                MissionState.TAKEOFF,
                MissionState.HOVER_STABILIZE,
                MissionState.SNAPSHOT_FORMATION,
                MissionState.WAITING_LEADER_COMMAND,
            }
            or (
                self.mission_manager.state
                in {
                    MissionState.LEADER_REPOSITION,
                    MissionState.FOLLOWERS_APPROACH,
                    MissionState.FORMATION_HOLD,
                    MissionState.FORMATION,
                }
                and (
                    leader_check is None
                    or leader_check.healthy
                )
            )
        )

        if should_check_safety:
            safety_result = self.safety_manager.check_states(
                states,
                now_ns,
            )

            if not safety_result.safe:
                reason = f'Safety failure: {safety_result.message}'
                self.handle_safety_failure(safety_result)
                if self.mission_manager.request_fault_landing(
                    reason
                ):
                    self.get_logger().error(
                        'Safety limit exceeded; landing all UAVs'
                    )

        output = self.mission_manager.update(
            states=states,
            now_ns=now_ns,
            controller=self.controller,
            leader_manager=self.leader_manager,
        )

        if (
            self.control_mode == CONTROL_MODE_CENTRALIZED
            and self.mission_manager.state in {
                MissionState.LEADER_REPOSITION,
                MissionState.FOLLOWERS_APPROACH,
                MissionState.FORMATION_HOLD,
            }
        ):
            staged_setpoints = self.centralized_staged_setpoints(
                states,
                now_ns,
            )
            if staged_setpoints:
                output.publish_offboard = True
                output.setpoints = staged_setpoints

        if (
            self.mission_manager.state in {
                MissionState.TAKEOFF,
                MissionState.HOVER_STABILIZE,
                MissionState.SNAPSHOT_FORMATION,
                MissionState.WAITING_LEADER_COMMAND,
                MissionState.LEADER_REPOSITION,
                MissionState.FOLLOWERS_APPROACH,
                MissionState.FORMATION_HOLD,
                MissionState.FORMATION,
            }
            and output.setpoints
        ):
            setpoint_safety = (
                self.safety_manager.check_setpoints(
                    states,
                    output.setpoints,
                )
            )
            if not setpoint_safety.safe:
                reason = (
                    'Unsafe formation setpoint: '
                    f'{setpoint_safety.message}'
                )
                self.handle_safety_failure(setpoint_safety)
                if self.mission_manager.request_fault_landing(
                    reason
                ):
                    output = self.mission_manager.update(
                        states=states,
                        now_ns=now_ns,
                        controller=self.controller,
                        leader_manager=self.leader_manager,
                    )

        # Mission-layer control ownership:
        #
        # OFFBOARD_WARMUP / TAKEOFF:
        #     mission_node owns all UAV heartbeat and setpoints.
        #
        # FORMATION + distributed:
        #     distributed_vehicle_node owns vehicle setpoints, so the
        #     mission layer must not publish competing heartbeat.
        #
        # Future leader_follower mode:
        #     leader_control_node will own leader setpoints and
        #     follower_control_node will own follower setpoints.
        skip_ground_heartbeat = (
            self.external_formation_controller_active()
        )

        if output.publish_offboard and not skip_ground_heartbeat:
            for vehicle_id, vehicle in self.vehicles.items():
                setpoint = output.setpoints.get(vehicle_id)
                use_velocity = bool(
                    setpoint is not None
                    and setpoint.velocity_local_enu is not None
                )
                vehicle.publish_offboard_heartbeat(
                    use_velocity=use_velocity
                )

        for vehicle_id, setpoint in output.setpoints.items():
            self.vehicles[vehicle_id].publish_setpoint(setpoint)

        if output.set_offboard_and_arm:
            self.arm_retry_start_ns = now_ns
            self.last_arm_retry_ns = now_ns
            self.arm_retry_timeout_reported = False

            for vehicle in self.vehicles.values():
                vehicle.set_offboard_mode()

            for vehicle in self.vehicles.values():
                vehicle.arm()

        self.retry_offboard_and_arm_if_needed(now_ns)

        if output.land_all:
            for vehicle in self.vehicles.values():
                vehicle.land()

        if self.last_mission_state != self.mission_manager.state:
            previous_state = self.last_mission_state
            self.get_logger().info(
                'Mission state: '
                f'{previous_state.name} -> '
                f'{self.mission_manager.state.name}'
            )
            self.last_mission_state = self.mission_manager.state

            if self.mission_manager.state in {
                MissionState.WAITING_READY,
                MissionState.OFFBOARD_WARMUP,
                MissionState.TAKEOFF,
            }:
                self.log_takeoff_diagnostics(
                    f'TAKEOFF STATE {self.mission_manager.state.name}'
                )
                self.last_takeoff_diagnostic_ns = now_ns

        self.maybe_log_takeoff_diagnostics(now_ns)


    def required_flight_state_failure(self, states):
        failures = []

        for vehicle_id, state in sorted(states.items()):
            if not state.status_received:
                failures.append(f'MAV{vehicle_id}:status_missing')
                continue

            if not state.armed:
                failures.append(f'MAV{vehicle_id}:not_armed')

            if not state.offboard_enabled:
                failures.append(f'MAV{vehicle_id}:offboard_inactive')

        return '; '.join(failures)

    def maybe_log_takeoff_diagnostics(self, now_ns):
        if self.mission_manager.state not in {
            MissionState.WAITING_READY,
            MissionState.OFFBOARD_WARMUP,
            MissionState.TAKEOFF,
        }:
            self.last_takeoff_diagnostic_ns = 0
            return

        if self.last_takeoff_diagnostic_ns == 0:
            self.last_takeoff_diagnostic_ns = now_ns
            return

        if (
            now_ns - self.last_takeoff_diagnostic_ns
            < self.takeoff_diagnostic_interval * 1e9
        ):
            return

        self.last_takeoff_diagnostic_ns = now_ns
        self.log_takeoff_diagnostics('TAKEOFF DIAGNOSTICS')

    def position_age_text(self, state, now_ns):
        if not state.position_received:
            return 'N/A'

        age_seconds = (
            now_ns - state.last_position_update_ns
        ) / 1e9
        return f'{age_seconds:.2f}s'

    def log_takeoff_diagnostics(self, title):
        now_ns = self.get_clock().now().nanoseconds
        lines = [
            '',
            f'========== {title} ==========',
        ]

        for vehicle_id, vehicle in sorted(self.vehicles.items()):
            state = vehicle.get_state()
            reason_text = vehicle.diagnostic_reason_text()
            advisory_text = vehicle.advisory_reason_text()

            lines.extend([
                f'MAV{vehicle_id}:',
                f'  status_received: {state.status_received}',
                f'  position_received: {state.position_received}',
                f'  position_age: {self.position_age_text(state, now_ns)}',
                f'  preflight_ok: {state.preflight_ok}',
                f'  armed: {state.armed}',
                f'  offboard_enabled: {state.offboard_enabled}',
                f'  nav_state: {state.nav_state}',
                f'  landed: {state.landed}',
                f'  position_valid: {state.position_valid}',
                f'  failsafe_received: {state.failsafe_received}',
                f'  health_report_received: '
                f'{state.health_report_received}',
                f'  diagnostic: {reason_text}',
                f'  advisory: {advisory_text}',
            ])

            if state.command_ack_received:
                lines.extend([
                    f'  last_command_ack: '
                    f'{state.last_command_ack_name}',
                    f'  last_command_result: '
                    f'{state.last_command_ack_result_name}',
                    f'  last_command_result_param1: '
                    f'{state.last_command_ack_result_param1}',
                    f'  last_command_result_param2: '
                    f'{state.last_command_ack_result_param2}',
                ])
            else:
                lines.append('  last_command_ack: N/A')

        lines.append('====================================')
        self.get_logger().info('\n'.join(lines))

    def retry_offboard_and_arm_if_needed(self, now_ns):
        if self.mission_manager.state != MissionState.TAKEOFF:
            self.arm_retry_start_ns = 0
            self.last_arm_retry_ns = 0
            self.arm_retry_timeout_reported = False
            return

        pending = {}
        for vehicle_id, vehicle in self.vehicles.items():
            state = vehicle.get_state()
            if not state.armed or not state.offboard_enabled:
                pending[vehicle_id] = state

        if not pending:
            self.arm_retry_start_ns = 0
            self.last_arm_retry_ns = 0
            self.arm_retry_timeout_reported = False
            return

        if self.arm_retry_start_ns == 0:
            self.arm_retry_start_ns = now_ns

        elapsed_seconds = (
            now_ns - self.arm_retry_start_ns
        ) / 1e9

        if (
            elapsed_seconds >= self.arm_retry_timeout
            and not self.arm_retry_timeout_reported
        ):
            pending_text = ', '.join(
                f'MAV{vehicle_id}:'
                f'armed={state.armed},'
                f'offboard={state.offboard_enabled}'
                for vehicle_id, state in sorted(pending.items())
            )
            reason = (
                'Offboard/arm timeout during takeoff: '
                f'{pending_text}'
            )
            self.arm_retry_timeout_reported = True
            if self.mission_manager.request_fault_landing(reason):
                self.get_logger().error(
                    reason + '; landing all UAVs'
                )
            return

        if (
            now_ns - self.last_arm_retry_ns
            < self.arm_retry_interval * 1e9
        ):
            return

        self.last_arm_retry_ns = now_ns

        pending_text = ', '.join(
            f'MAV{vehicle_id}:'
            f'armed={state.armed},'
            f'offboard={state.offboard_enabled}'
            for vehicle_id, state in sorted(pending.items())
        )
        self.get_logger().warning(
            'Retrying OFFBOARD/ARM for pending UAVs: '
            f'{pending_text}'
        )

        for vehicle_id in sorted(pending):
            vehicle = self.vehicles[vehicle_id]
            state = pending[vehicle_id]
            if not state.offboard_enabled:
                vehicle.set_offboard_mode()
            if not state.armed:
                vehicle.arm()

    def follower_status_callback(self, message):
        if int(message.leader_id) != self.leader_manager.leader_id:
            return
        if (
            int(message.leader_generation)
            != self.leader_manager.leader_generation
        ):
            return

        self.follower_control_ready = bool(
            message.formation_ready
        )
        self.follower_maximum_position_error = float(
            message.maximum_position_error
        )

        if (
            self.follower_control_ready
            and not self.formation_complete_reported
            and self.mission_manager.state == MissionState.FORMATION
        ):
            self.get_logger().info(
                'FORMATION COMPLETE: landing command may be sent'
            )
            self.formation_complete_reported = True

    def command_callback(self, message):
        if int(message.leader_id) != self.leader_manager.leader_id:
            return

        if (
            int(message.leader_generation)
            != self.leader_manager.leader_generation
        ):
            return

        self.latest_command = message
        self.last_command_receive_ns = (
            self.get_clock().now().nanoseconds
        )
        self.command_timeout_reported = False

    def apply_command_formation_settings(self, command):
        formation_type = str(command.formation_type).strip()
        formation_changed = False

        if formation_type:
            if formation_type != self.formation_type:
                self.formation_type = formation_type
                formation_changed = True

        if command.spacing > 0.0:
            new_spacing = float(command.spacing)
            if abs(new_spacing - self.formation_spacing) > 1e-6:
                self.formation_spacing = new_spacing
                formation_changed = True

        if (
            formation_changed
            and self.control_mode == CONTROL_MODE_CENTRALIZED
            and hasattr(self.controller, 'set_formation')
        ):
            self.controller.set_formation(
                self.formation_type,
                self.formation_spacing,
            )

        if formation_changed:
            self.get_logger().info(
                'Formation command updated shape to '
                f'{self.formation_type}, spacing '
                f'{self.formation_spacing:.2f} m'
            )

    def process_formation_command(self, now_ns):
        if self.latest_command is None:
            return

        age_seconds = (
            now_ns - self.last_command_receive_ns
        ) / 1e9
        if age_seconds > self.command_timeout:
            # Waiting for a leader command is not a fault.  It is a
            # normal mission state after takeoff/snapshot.
            if self.mission_manager.state in {
                MissionState.WAITING_LEADER_COMMAND,
                MissionState.FORMATION_HOLD,
            }:
                return

            if not self.command_timeout_reported:
                reason = 'Formation command timeout'
                if self.mission_manager.request_fault_landing(
                    reason
                ):
                    self.get_logger().error(
                        reason + '; landing all UAVs'
                    )
                self.command_timeout_reported = True
            return

        command = self.latest_command
        self.apply_command_formation_settings(command)

        if command.command == FormationCommand.MISSION_COMPLETE:
            if self.mission_manager.request_land():
                self.get_logger().info(
                    'Mission complete command received; landing all UAVs'
                )
            return

        if command.command == FormationCommand.EMERGENCY_STOP:
            reason = 'Emergency stop command received'
            if self.mission_manager.request_fault_landing(reason):
                self.get_logger().error(
                    reason + '; landing all UAVs'
                )
            return

        if self.control_mode == CONTROL_MODE_CENTRALIZED:
            self.process_centralized_command(command, now_ns)
            return

        if (
            self.mission_manager.state
            == MissionState.WAITING_LEADER_COMMAND
        ):
            if command.command != FormationCommand.MOVE:
                return

            if self.mission_manager.request_start_formation():
                self.follower_control_ready = False
                self.follower_maximum_position_error = 0.0
                self.formation_complete_reported = False
                self.get_logger().info(
                    'Leader MOVE command received; starting FORMATION'
                )
            return

        if self.control_mode != CONTROL_MODE_CENTRALIZED:
            return

    def process_centralized_command(self, command, now_ns):
        if command.command != FormationCommand.MOVE:
            return

        if self.mission_manager.state not in {
            MissionState.WAITING_LEADER_COMMAND,
            MissionState.FORMATION_HOLD,
        }:
            return

        sequence = self.command_sequence(command)
        if self.centralized_active_move_sequence == sequence:
            return

        states = {
            vehicle_id: vehicle.get_state()
            for vehicle_id, vehicle in self.vehicles.items()
        }
        if not self.validate_centralized_move_command(states, command):
            return

        if not self.mission_manager.request_leader_reposition(
            sequence=sequence,
            duration=float(command.duration),
            now_ns=now_ns,
        ):
            return

        self.centralized_active_move_sequence = sequence
        self.centralized_active_velocity = VectorENU(
            east=float(command.velocity_east),
            north=float(command.velocity_north),
            up=float(command.velocity_up),
        )
        self.centralized_formation_ready_since_ns = None
        self.formation_complete_reported = False

        self.get_logger().info(
            f'Leader MOVE sequence {sequence} accepted; '
            'mission state entering LEADER_REPOSITION'
        )

    def external_formation_controller_active(self):
        """Return True when another node owns FORMATION setpoints."""

        return (
            self.control_mode == CONTROL_MODE_DISTRIBUTED
            and self.mission_manager.state == MissionState.FORMATION
        )

    def takeoff_callback(self, request, response):
        del request

        response.success = self.mission_manager.request_takeoff()
        if response.success:
            self.centralized_offsets = None
            self.centralized_active_move_sequence = None
            self.centralized_active_velocity = VectorENU(0.0, 0.0, 0.0)
            self.centralized_maximum_position_error = 0.0
            self.centralized_maximum_speed = 0.0
            self.centralized_formation_ready_since_ns = None
            self.formation_complete_reported = False
            self.log_takeoff_diagnostics('TAKEOFF REQUESTED')

        response.message = (
            'Takeoff sequence started'
            if response.success
            else 'Takeoff rejected in state '
            f'{self.mission_manager.state.name}'
        )
        return response

    def land_callback(self, request, response):
        del request

        response.success = self.mission_manager.request_land()
        response.message = (
            'Landing sequence started'
            if response.success
            else 'Landing rejected in state '
            f'{self.mission_manager.state.name}'
        )
        return response

    def publish_formation_status(self):
        now = self.get_clock().now()
        states = {
            vehicle_id: vehicle.get_state()
            for vehicle_id, vehicle in self.vehicles.items()
        }

        leader_health = self.leader_manager.evaluate_vehicle(
            vehicle_id=self.leader_manager.leader_id,
            states=states,
            now_ns=now.nanoseconds,
            require_armed=(
                self.mission_manager.state
                in {
                    MissionState.LEADER_REPOSITION,
                    MissionState.FOLLOWERS_APPROACH,
                    MissionState.FORMATION_HOLD,
                    MissionState.FORMATION,
                }
            ),
            require_offboard=(
                self.mission_manager.state
                in {
                    MissionState.LEADER_REPOSITION,
                    MissionState.FOLLOWERS_APPROACH,
                    MissionState.FORMATION_HOLD,
                    MissionState.FORMATION,
                }
            ),
            require_preflight=(
                self.mission_manager.state
                not in {
                    MissionState.LEADER_REPOSITION,
                    MissionState.FOLLOWERS_APPROACH,
                    MissionState.FORMATION_HOLD,
                    MissionState.FORMATION,
                }
            ),
        )

        message = FormationStatus()
        message.stamp = now.to_msg()
        message.leader_id = self.leader_manager.leader_id
        message.mission_state = (
            self.mission_manager.state.name
        )
        message.control_mode = self.control_mode
        message.formation_type = self.formation_type
        message.spacing = float(self.formation_spacing)
        if self.control_mode == CONTROL_MODE_CENTRALIZED:
            maximum_position_error = (
                self.centralized_maximum_position_error
            )
            message.formation_ready = (
                self.mission_manager.state
                == MissionState.FORMATION_HOLD
            )
        elif self.control_mode == CONTROL_MODE_LEADER_FOLLOWER:
            maximum_position_error = (
                self.follower_maximum_position_error
            )
            message.formation_ready = (
                self.mission_manager.state
                in {
                    MissionState.FORMATION_HOLD,
                    MissionState.FORMATION,
                }
                and self.follower_control_ready
            )
        else:
            maximum_position_error = float(
                getattr(
                    self.controller,
                    'maximum_position_error',
                    0.0,
                )
            )
            message.formation_ready = (
                self.mission_manager.state == MissionState.FORMATION
                and maximum_position_error
                <= self.formation_position_tolerance
            )
        message.maximum_position_error = (
            maximum_position_error
        )

        message.leader_healthy = leader_health.healthy
        message.leader_fault_reason = (
            self.leader_manager.last_confirmed_fault_reason
            or (
                ''
                if leader_health.healthy
                else leader_health.reason
            )
        )
        message.leader_generation = (
            self.leader_manager.leader_generation
        )
        message.failed_leader_id = (
            self.leader_manager.failed_leader_id or 0
        )
        message.awaiting_ground_confirmation = (
            self.mission_manager.state
            == MissionState.AWAITING_GROUND_CONFIRMATION
        )

        self.status_publisher.publish(message)

    def set_leader_callback(self, request, response):
        allowed_states = {
            MissionState.IDLE,
            MissionState.LANDED,
            MissionState.AWAITING_GROUND_CONFIRMATION,
        }

        if self.mission_manager.state not in allowed_states:
            response.success = False
            response.message = (
                'Leader change rejected in state '
                f'{self.mission_manager.state.name}'
            )
            return response

        leader_id = int(request.leader_id)
        state = self.vehicles.get(leader_id)

        if state is None:
            response.success = False
            response.message = (
                f'Unknown leader ID {leader_id}'
            )
            return response

        vehicle_state = state.get_state()

        if not (
            vehicle_state.land_status_received
            and vehicle_state.landed
        ):
            response.success = False
            response.message = (
                f'Vehicle {leader_id} is not confirmed landed'
            )
            return response

        now_ns = self.get_clock().now().nanoseconds
        health = self.leader_manager.evaluate_vehicle(
            vehicle_id=leader_id,
            states={
                vehicle_id: vehicle.get_state()
                for vehicle_id, vehicle
                in self.vehicles.items()
            },
            now_ns=now_ns,
        )

        if not health.healthy:
            response.success = False
            response.message = (
                f'Vehicle {leader_id} is unhealthy: '
                f'{health.reason}'
            )
            return response

        response.success = self.leader_manager.set_leader(
            leader_id=leader_id,
            available_ids=set(self.vehicles),
        )
        if response.success:
            self.centralized_offsets = None
            self.centralized_active_move_sequence = None
            self.centralized_formation_ready_since_ns = None
            self.formation_complete_reported = False

        response.message = (
            f'Leader set to vehicle {leader_id}'
            if response.success
            else f'Failed to set leader {leader_id}'
        )

        if response.success:
            self.get_logger().info(response.message)

        return response

    def confirm_fault_callback(self, request, response):
        del request

        if (
            self.mission_manager.state
            != MissionState.AWAITING_GROUND_CONFIRMATION
        ):
            response.success = False
            response.message = (
                'Confirmation rejected in state '
                f'{self.mission_manager.state.name}'
            )
            return response

        states = {
            vehicle_id: vehicle.get_state()
            for vehicle_id, vehicle in self.vehicles.items()
        }
        now_ns = self.get_clock().now().nanoseconds

        if not self.mission_manager.all_vehicles_landed(states):
            response.success = False
            response.message = (
                'Confirmation rejected: not all UAVs are landed'
            )
            return response

        if not self.mission_manager.all_vehicles_ready(
            states,
            now_ns,
        ):
            response.success = False
            response.message = (
                'Confirmation rejected: UAV readiness '
                'has not recovered'
            )
            return response

        leader_health = self.leader_manager.evaluate_vehicle(
            vehicle_id=self.leader_manager.leader_id,
            states=states,
            now_ns=now_ns,
        )

        if not leader_health.healthy:
            response.success = False
            response.message = (
                'Confirmation rejected: current leader '
                f'is unhealthy ({leader_health.reason})'
            )
            return response

        response.success = (
            self.mission_manager.confirm_fault_recovery()
        )

        if response.success:
            self.leader_manager.clear_confirmed_fault()
            self.last_leader_warning_reason = ''

        response.message = (
            'Fault acknowledged; mission reset to LANDED'
            if response.success
            else 'Fault confirmation failed'
        )
        return response

    def handle_safety_failure(self, result):
        self.get_logger().error(result.message)


FormationNode = MissionNode


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # rclpy may raise RCLError when launch/timeout has already
        # invalidated the context. Re-raise genuine runtime failures.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
