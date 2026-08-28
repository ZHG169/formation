from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class VehicleHealth:
    healthy: bool
    reason: str = ''
    position_age_seconds: Optional[float] = None


@dataclass(frozen=True)
class LeaderCheck:
    healthy: bool
    failure_confirmed: bool
    reason: str = ''
    unhealthy_duration: float = 0.0


class LeaderManager:

    def __init__(
        self,
        initial_leader_id,
        position_timeout=2.0,
        failure_hold_duration=1.0,
    ):
        self.leader_id = int(initial_leader_id)
        self.position_timeout = float(position_timeout)
        self.failure_hold_duration = float(
            failure_hold_duration
        )

        self.failure_since_ns = None
        self.last_failure_reason = ''

        self.leader_generation = 0
        self.failed_leader_id = None
        self.last_confirmed_fault_reason = ''
        self.last_confirmed_fault_time_ns = None

    def set_leader(self, leader_id, available_ids):
        leader_id = int(leader_id)

        if leader_id not in available_ids:
            return False

        if leader_id != self.leader_id:
            self.leader_id = leader_id
            self.leader_generation += 1

        self._reset_failure_tracking()
        return True

    def evaluate_vehicle(
        self,
        vehicle_id,
        states,
        now_ns,
        require_armed=False,
        require_offboard=False,
        require_preflight=True,
    ):
        state = states.get(vehicle_id)

        if state is None:
            return VehicleHealth(False, 'state_missing')

        if not state.status_received:
            return VehicleHealth(False, 'status_missing')

        if require_preflight and not state.preflight_ok:
            reason = getattr(state, 'preflight_reason', '')
            if not reason:
                health_error_flags = getattr(
                    state,
                    'health_error_flags',
                    0,
                )
                arming_check_error_flags = getattr(
                    state,
                    'arming_check_error_flags',
                    0,
                )
                extra = []
                if health_error_flags:
                    extra.append(
                        f'health_error_flags={health_error_flags}'
                    )
                if arming_check_error_flags:
                    extra.append(
                        'arming_check_error_flags='
                        f'{arming_check_error_flags}'
                    )
                reason = ','.join(extra)

            if reason:
                return VehicleHealth(
                    False,
                    f'preflight_failed:{reason}',
                )

            return VehicleHealth(False, 'preflight_failed')

        if not state.position_received:
            return VehicleHealth(False, 'position_missing')

        if (
            not state.position_valid
            or state.position_local_enu is None
        ):
            return VehicleHealth(False, 'position_invalid')

        position_age = (
            now_ns - state.last_position_update_ns
        ) / 1e9

        if position_age < 0.0:
            return VehicleHealth(
                False,
                'clock_moved_backwards',
                position_age,
            )

        if position_age > self.position_timeout:
            return VehicleHealth(
                False,
                'position_stale',
                position_age,
            )

        if require_armed and not state.armed:
            return VehicleHealth(
                False,
                'not_armed',
                position_age,
            )

        if require_offboard and not state.offboard_enabled:
            return VehicleHealth(
                False,
                'offboard_inactive',
                position_age,
            )

        return VehicleHealth(
            True,
            position_age_seconds=position_age,
        )

    def leader_is_available(
        self,
        states,
        now_ns,
        require_armed=False,
        require_offboard=False,
        require_preflight=True,
    ):
        return self.evaluate_vehicle(
            self.leader_id,
            states,
            now_ns,
            require_armed=require_armed,
            require_offboard=require_offboard,
            require_preflight=require_preflight,
        ).healthy

    def check_leader(
        self,
        states,
        now_ns,
        require_armed=False,
        require_offboard=False,
        require_preflight=True,
    ):
        health = self.evaluate_vehicle(
            self.leader_id,
            states,
            now_ns,
            require_armed=require_armed,
            require_offboard=require_offboard,
            require_preflight=require_preflight,
        )

        if health.healthy:
            self._reset_failure_tracking()
            return LeaderCheck(True, False)

        if (
            self.failure_since_ns is None
            or health.reason != self.last_failure_reason
        ):
            self.failure_since_ns = now_ns
            self.last_failure_reason = health.reason

        unhealthy_duration = max(
            0.0,
            (now_ns - self.failure_since_ns) / 1e9,
        )

        failure_confirmed = (
            unhealthy_duration
            >= self.failure_hold_duration
        )

        if failure_confirmed:
            self.failed_leader_id = self.leader_id
            self.last_confirmed_fault_reason = health.reason
            self.last_confirmed_fault_time_ns = now_ns

        return LeaderCheck(
            healthy=False,
            failure_confirmed=failure_confirmed,
            reason=health.reason,
            unhealthy_duration=unhealthy_duration,
        )

    def elect_new_leader(
        self,
        states,
        now_ns,
        require_armed=False,
        require_offboard=False,
        require_preflight=True,
        exclude_current=True,
    ):
        candidates = []

        for vehicle_id in sorted(states):
            if exclude_current and vehicle_id == self.leader_id:
                continue

            health = self.evaluate_vehicle(
                vehicle_id,
                states,
                now_ns,
                require_armed=require_armed,
                require_offboard=require_offboard,
                require_preflight=require_preflight,
            )

            if health.healthy:
                candidates.append(vehicle_id)

        if not candidates:
            return None

        if candidates[0] != self.leader_id:
            self.leader_id = candidates[0]
            self.leader_generation += 1

        self._reset_failure_tracking()
        return self.leader_id

    def clear_confirmed_fault(self):
        self.failed_leader_id = None
        self.last_confirmed_fault_reason = ''
        self.last_confirmed_fault_time_ns = None
        self._reset_failure_tracking()

    def _reset_failure_tracking(self):
        self.failure_since_ns = None
        self.last_failure_reason = ''
