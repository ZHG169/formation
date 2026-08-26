import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

from formation.msg import FormationCommand, FormationStatus
from formation.coordinate_convert import VectorENU
from formation.cpf_avoidance import CpfConfig
from formation.srv import SetLeader
from formation.formation_controller import (
    CentralizedFormationController,
    DistributedFormationController,
    FormationReference,
)
from formation.leader_manager import LeaderManager
from formation.mission_manager import MissionManager, MissionState
from formation.safety_manager import SafetyManager
from formation.vehicle_interface import VehicleInterface

"""
Mission-layer node for the formation package.

Current responsibility:
    mission_node
        Owns mission state, synchronized takeoff, synchronized landing,
        emergency landing, leader selection, safety checks and status.

Next split names:
    leader_control_node
        Future node that owns only the leader setpoint during FORMATION.

    follower_formation_node
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
                ('formation_reference_enu', [
                    0.0, 0.0, 3.0,
                ]),
                ('formation_reference_yaw_enu', 0.0),
                ('command_timeout', 1.0),
                ('arm_retry_interval', 0.5),
                ('arm_retry_timeout', 8.0),
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

        self.follower_formation_ready = False
        self.follower_maximum_position_error = 0.0
        self.formation_complete_reported = False

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

    def control_loop(self):
        states = {
            vehicle_id: vehicle.get_state()
            for vehicle_id, vehicle in self.vehicles.items()
        }

        now_ns = self.get_clock().now().nanoseconds

        leader_check = None

        if self.mission_manager.state in {
            MissionState.WAITING_LEADER_COMMAND,
            MissionState.FORMATION,
        }:
            self.process_formation_command(now_ns)

        if self.mission_manager.state == MissionState.FORMATION:
            leader_check = self.leader_manager.check_leader(
                states=states,
                now_ns=now_ns,
                require_armed=True,
                require_offboard=True,
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
                MissionState.WAITING_LEADER_COMMAND,
            }
            or (
                self.mission_manager.state
                == MissionState.FORMATION
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
            self.mission_manager.state in {
                MissionState.TAKEOFF,
                MissionState.WAITING_LEADER_COMMAND,
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
        #     follower_formation_node will own follower setpoints.
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
            self.get_logger().info(
                'Mission state: '
                f'{self.last_mission_state.name} -> '
                f'{self.mission_manager.state.name}'
            )
            self.last_mission_state = self.mission_manager.state


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

        self.follower_formation_ready = bool(
            message.formation_ready
        )
        self.follower_maximum_position_error = float(
            message.maximum_position_error
        )

        if (
            self.follower_formation_ready
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

        if (
            self.mission_manager.state
            == MissionState.WAITING_LEADER_COMMAND
        ):
            if command.command != FormationCommand.MOVE:
                return

            if self.mission_manager.request_start_formation():
                self.follower_formation_ready = False
                self.follower_maximum_position_error = 0.0
                self.formation_complete_reported = False
                self.get_logger().info(
                    'Leader MOVE command received; starting FORMATION'
                )
            return

        if command.command == FormationCommand.EMERGENCY_STOP:
            reason = 'Emergency stop command received'
            if self.mission_manager.request_fault_landing(reason):
                self.get_logger().error(
                    reason + '; landing all UAVs'
                )
            return

        if self.control_mode != 'centralized':
            return

        if command.command == FormationCommand.MOVE:
            velocity = VectorENU(
                east=float(command.velocity_east),
                north=float(command.velocity_north),
                up=float(command.velocity_up),
            )
            self.controller.apply_velocity_command(
                velocity_enu=velocity,
                yaw_rate=float(command.yaw_rate),
                dt_seconds=self.control_period,
            )
        elif command.command == FormationCommand.SET_YAW:
            self.controller.set_reference_yaw(
                float(command.yaw_enu)
            )

    def external_formation_controller_active(self):
        """Return True when another node owns FORMATION setpoints."""

        return (
            self.control_mode in {
                CONTROL_MODE_DISTRIBUTED,
                CONTROL_MODE_LEADER_FOLLOWER,
            }
            and self.mission_manager.state == MissionState.FORMATION
        )

    def takeoff_callback(self, request, response):
        del request

        response.success = self.mission_manager.request_takeoff()
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
                == MissionState.FORMATION
            ),
            require_offboard=(
                self.mission_manager.state
                == MissionState.FORMATION
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
        if self.control_mode == CONTROL_MODE_LEADER_FOLLOWER:
            maximum_position_error = (
                self.follower_maximum_position_error
            )
            message.formation_ready = (
                self.mission_manager.state
                == MissionState.FORMATION
                and self.follower_formation_ready
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
                self.mission_manager.state
                == MissionState.FORMATION
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
