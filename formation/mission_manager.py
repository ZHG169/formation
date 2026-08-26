from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict

from formation.coordinate_convert import VectorENU
from formation.vehicle_interface import VehicleSetpoint


class MissionState(Enum):
    IDLE = auto()
    WAITING_READY = auto()
    OFFBOARD_WARMUP = auto()
    TAKEOFF = auto()
    WAITING_LEADER_COMMAND = auto()
    FORMATION = auto()
    LANDING = auto()
    LANDED = auto()
    EMERGENCY_LANDING = auto()
    AWAITING_GROUND_CONFIRMATION = auto()
    ERROR = auto()


# 狀態每次回傳
@dataclass
class MissionOutput:
    setpoints: Dict[int, VehicleSetpoint] = field(
        default_factory=dict
    )
    publish_offboard: bool = False
    set_offboard_and_arm: bool = False
    land_all: bool = False


class MissionManager:

    def __init__(
        self,
        takeoff_height=3.0,
        ready_hold_duration=2.0, # 狀態確認
        warmup_duration=2.0, # 無人機起飛前的等待
        position_timeout=2.0,
        altitude_tolerance=0.3,
        land_command_interval=1.0,
        liftoff_after_arm_timeout=10.0,
        takeoff_climb_rate=0.25,
    ):
        self.state = MissionState.IDLE

        self.takeoff_height = float(takeoff_height)
        self.ready_hold_duration = float(ready_hold_duration)
        self.warmup_duration = float(warmup_duration)
        self.position_timeout = float(position_timeout)
        self.altitude_tolerance = float(altitude_tolerance)
        self.land_command_interval = float(
            land_command_interval
        )
        self.liftoff_after_arm_timeout = float(
            liftoff_after_arm_timeout
        )
        self.takeoff_climb_rate = float(takeoff_climb_rate)

        self.ready_since_ns = None
        self.warmup_start_ns = None
        self.takeoff_setpoints = {}
        self.takeoff_home_setpoints = {}
        self.takeoff_target_setpoints = {}
        self.liftoff_authorized = False
        self.armed_wait_start_ns = None
        self.takeoff_ramp_start_ns = None
        self.land_command_pending = False
        self.last_land_command_ns = None
        self.fault_reason = ''

    def request_takeoff(self):
        if self.state not in {
            MissionState.IDLE,
            MissionState.LANDED,
        }:
            return False

        self.state = MissionState.WAITING_READY
        self.ready_since_ns = None
        self.warmup_start_ns = None
        self.takeoff_setpoints = {}
        self.takeoff_home_setpoints = {}
        self.takeoff_target_setpoints = {}
        self.liftoff_authorized = False
        self.armed_wait_start_ns = None
        self.takeoff_ramp_start_ns = None
        self.land_command_pending = False
        self.last_land_command_ns = None
        self.fault_reason = ''
        return True

    def request_land(self):
        if self.state not in {
            MissionState.TAKEOFF,
            MissionState.WAITING_LEADER_COMMAND,
            MissionState.FORMATION,
        }:
            return False

        self.state = MissionState.LANDING
        self.land_command_pending = True
        self.last_land_command_ns = None
        return True

    def request_fault_landing(self, reason):
        if self.state not in {
            MissionState.TAKEOFF,
            MissionState.WAITING_LEADER_COMMAND,
            MissionState.FORMATION,
        }:
            return False

        self.state = MissionState.EMERGENCY_LANDING
        self.fault_reason = str(reason)
        self.land_command_pending = True
        self.last_land_command_ns = None
        return True

    def confirm_fault_recovery(self):
        if (
            self.state
            != MissionState.AWAITING_GROUND_CONFIRMATION
        ):
            return False

        self.state = MissionState.LANDED
        self.fault_reason = ''
        return True

    def requires_offboard(self):
        return self.state in {
            MissionState.OFFBOARD_WARMUP,
            MissionState.TAKEOFF,
            MissionState.WAITING_LEADER_COMMAND,
            MissionState.FORMATION,
        }

    def set_error(self):
        self.state = MissionState.ERROR

    def request_start_formation(self):
        if self.state != MissionState.WAITING_LEADER_COMMAND:
            return False

        self.state = MissionState.FORMATION
        return True

    def _state_is_ready(self, state, now_ns):
        if not (
            state.status_received
            and state.preflight_ok
            and state.position_received
            and state.position_valid
            and state.position_local_enu is not None
        ):
            return False

        age_seconds = (
            now_ns - state.last_position_update_ns
        ) / 1e9

        return 0.0 <= age_seconds <= self.position_timeout

    def _all_ready(self, states, now_ns):
        return bool(states) and all(
            self._state_is_ready(state, now_ns)
            for state in states.values()
        )

    # Capture each vehicle's current horizontal position as its
    # takeoff point, then command only the target altitude.
    #
    # This keeps the UAVs from flying back to local (0, 0) during
    # takeoff.  It also allows the same code path to work for both:
    #   - Gazebo: local position + YAML spawn origin
    #   - real flight: PX4 local_position from OptiTrack/external vision
    def _capture_takeoff_setpoints(self, states):
        self.takeoff_home_setpoints = {}
        self.takeoff_target_setpoints = {}

        for vehicle_id, state in states.items():
            home = VehicleSetpoint(
                position_local_enu=VectorENU(
                    east=state.position_local_enu.east,
                    north=state.position_local_enu.north,
                    up=state.position_local_enu.up,
                ),
                yaw_local_enu=state.yaw_local_enu,
            )
            target = VehicleSetpoint(
                position_local_enu=VectorENU(
                    east=state.position_local_enu.east,
                    north=state.position_local_enu.north,
                    up=self.takeoff_height,
                ),
                yaw_local_enu=state.yaw_local_enu,
            )
            self.takeoff_home_setpoints[vehicle_id] = home
            self.takeoff_target_setpoints[vehicle_id] = target

        self.takeoff_setpoints = dict(self.takeoff_home_setpoints)
        self.liftoff_authorized = False
        self.armed_wait_start_ns = None
        self.takeoff_ramp_start_ns = None

    def _all_armed_and_offboard(self, states):
        return bool(states) and all(
            state.armed and state.offboard_enabled
            for state in states.values()
        )

    def _ramped_takeoff_setpoints(self, now_ns):
        if self.takeoff_climb_rate <= 0.0:
            return dict(self.takeoff_target_setpoints)

        if self.takeoff_ramp_start_ns is None:
            self.takeoff_ramp_start_ns = now_ns

        elapsed = (now_ns - self.takeoff_ramp_start_ns) / 1e9
        climb_distance = self.takeoff_climb_rate * elapsed
        setpoints = {}

        for vehicle_id, target in self.takeoff_target_setpoints.items():
            home = self.takeoff_home_setpoints[vehicle_id]
            home_up = home.position_local_enu.up
            target_up = target.position_local_enu.up
            delta_up = target_up - home_up

            if abs(delta_up) <= 1e-6:
                commanded_up = target_up
            else:
                direction = 1.0 if delta_up > 0.0 else -1.0
                commanded_up = home_up + direction * min(
                    abs(delta_up),
                    climb_distance,
                )

            setpoints[vehicle_id] = VehicleSetpoint(
                position_local_enu=VectorENU(
                    east=target.position_local_enu.east,
                    north=target.position_local_enu.north,
                    up=commanded_up,
                ),
                yaw_local_enu=target.yaw_local_enu,
            )

        return setpoints

    def _all_at_takeoff_height(self, states):
        return bool(states) and all(
            state.armed
            and state.offboard_enabled
            and state.position_valid
            and state.position_local_enu is not None
            and abs(
                state.position_local_enu.up
                - self.takeoff_height
            ) <= self.altitude_tolerance
            for state in states.values()
        )

    @staticmethod
    def _all_landed(states):
        return bool(states) and all(
            state.land_status_received and state.landed
            for state in states.values()
        )

    def all_vehicles_ready(self, states, now_ns):
        return self._all_ready(states, now_ns)

    def all_vehicles_landed(self, states):
        return self._all_landed(states)

    def update(
        self,
        states,
        now_ns,
        controller,
        leader_manager,
    ):
        output = MissionOutput()

        if self.state == MissionState.WAITING_READY:
            if not self._all_ready(states, now_ns):
                self.ready_since_ns = None
                return output

            if self.ready_since_ns is None:
                self.ready_since_ns = now_ns
                return output

            ready_elapsed = (
                now_ns - self.ready_since_ns
            ) / 1e9

            if ready_elapsed >= self.ready_hold_duration:
                self._capture_takeoff_setpoints(states)
                self.warmup_start_ns = now_ns
                self.state = MissionState.OFFBOARD_WARMUP
                output.publish_offboard = True
                output.setpoints = dict(self.takeoff_home_setpoints)

            return output

        if self.state == MissionState.OFFBOARD_WARMUP:
            if not self._all_ready(states, now_ns):
                self.state = MissionState.WAITING_READY
                self.ready_since_ns = None
                self.warmup_start_ns = None
                self.takeoff_setpoints = {}
                self.takeoff_home_setpoints = {}
                self.takeoff_target_setpoints = {}
                self.liftoff_authorized = False
                self.armed_wait_start_ns = None
                self.takeoff_ramp_start_ns = None
                return output

            output.publish_offboard = True
            output.setpoints = dict(self.takeoff_home_setpoints)

            warmup_elapsed = (
                now_ns - self.warmup_start_ns
            ) / 1e9

            if warmup_elapsed >= self.warmup_duration:
                output.set_offboard_and_arm = True
                self.state = MissionState.TAKEOFF

            return output

        if self.state == MissionState.TAKEOFF:
            output.publish_offboard = True

            if not self.liftoff_authorized:
                output.setpoints = dict(self.takeoff_home_setpoints)

                if self.armed_wait_start_ns is None:
                    self.armed_wait_start_ns = now_ns

                elapsed = (
                    now_ns - self.armed_wait_start_ns
                ) / 1e9

                if (
                    self._all_armed_and_offboard(states)
                    or elapsed >= self.liftoff_after_arm_timeout
                ):
                    self.liftoff_authorized = True
                    self.takeoff_ramp_start_ns = now_ns
                    output.setpoints = self._ramped_takeoff_setpoints(
                        now_ns
                    )

                return output

            output.setpoints = self._ramped_takeoff_setpoints(now_ns)
            self.takeoff_setpoints = dict(output.setpoints)

            if self._all_at_takeoff_height(states):
                self.takeoff_setpoints = dict(
                    self.takeoff_target_setpoints
                )
                self.state = MissionState.WAITING_LEADER_COMMAND

            return output

        if self.state == MissionState.WAITING_LEADER_COMMAND:
            output.publish_offboard = True
            output.setpoints = dict(self.takeoff_target_setpoints)
            return output

        if self.state == MissionState.FORMATION:
            # FORMATION setpoints are owned by dedicated control nodes.
            # mission_node remains the state owner and keeps landing /
            # emergency authority, but it does not publish formation
            # heartbeat or setpoints in this state.
            del controller, leader_manager
            return output

        if self.state in {
            MissionState.LANDING,
            MissionState.EMERGENCY_LANDING,
        }:
            emergency = (
                self.state == MissionState.EMERGENCY_LANDING
            )

            if self._all_landed(states):
                self.state = (
                    MissionState.AWAITING_GROUND_CONFIRMATION
                    if emergency
                    else MissionState.LANDED
                )
                self.land_command_pending = False
                return output

            command_due = (
                self.last_land_command_ns is None
                or (
                    now_ns - self.last_land_command_ns
                ) / 1e9 >= self.land_command_interval
            )

            if self.land_command_pending or command_due:
                output.land_all = True
                self.land_command_pending = False
                self.last_land_command_ns = now_ns

            return output

        return output
