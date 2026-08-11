import math
from dataclasses import dataclass, field

from formation.coordinate_convert import VectorENU
from formation.formation_controller import (
    add_vectors,
    vector_distance,
)


@dataclass(frozen=True)
class SafetyViolation:
    code: str
    message: str
    vehicle_ids: tuple = ()


@dataclass
class SafetyResult:
    safe: bool
    critical: bool = False
    message: str = ''
    violations: list = field(default_factory=list)


class SafetyManager:

    def __init__(
        self,
        vehicle_origins,
        position_timeout=2.0,
        minimum_distance=1.0,
        maximum_altitude=20.0,
        maximum_speed=8.0,
        geofence_radius=50.0,
        maximum_setpoint_jump=5.0,
    ):
        self.vehicle_origins = dict(vehicle_origins)
        self.position_timeout = float(position_timeout)
        self.minimum_distance = float(minimum_distance)
        self.maximum_altitude = float(maximum_altitude)
        self.maximum_speed = float(maximum_speed)
        self.geofence_radius = float(geofence_radius)
        self.maximum_setpoint_jump = float(
            maximum_setpoint_jump
        )

    @staticmethod
    def _result(violations):
        if not violations:
            return SafetyResult(True)

        return SafetyResult(
            safe=False,
            critical=True,
            message='; '.join(
                violation.message
                for violation in violations
            ),
            violations=violations,
        )

    def _world_position(self, vehicle_id, local_position):
        return add_vectors(
            self.vehicle_origins[vehicle_id],
            local_position,
        )

    def check_states(self, states, now_ns):
        violations = []
        world_positions = {}

        for vehicle_id, state in states.items():
            if vehicle_id not in self.vehicle_origins:
                violations.append(
                    SafetyViolation(
                        'origin_missing',
                        f'Vehicle {vehicle_id} origin missing',
                        (vehicle_id,),
                    )
                )
                continue

            if (
                not state.position_received
                or not state.position_valid
                or state.position_local_enu is None
            ):
                violations.append(
                    SafetyViolation(
                        'position_invalid',
                        f'Vehicle {vehicle_id} position invalid',
                        (vehicle_id,),
                    )
                )
                continue

            age_seconds = (
                now_ns - state.last_position_update_ns
            ) / 1e9
            if not (
                0.0 <= age_seconds <= self.position_timeout
            ):
                violations.append(
                    SafetyViolation(
                        'position_stale',
                        f'Vehicle {vehicle_id} position stale',
                        (vehicle_id,),
                    )
                )
                continue

            world_position = self._world_position(
                vehicle_id,
                state.position_local_enu,
            )
            world_positions[vehicle_id] = world_position

            if (
                world_position.up < -0.5
                or world_position.up > self.maximum_altitude
            ):
                violations.append(
                    SafetyViolation(
                        'altitude_limit',
                        f'Vehicle {vehicle_id} altitude '
                        f'{world_position.up:.2f} m out of range',
                        (vehicle_id,),
                    )
                )

            horizontal_radius = math.hypot(
                world_position.east,
                world_position.north,
            )
            if horizontal_radius > self.geofence_radius:
                violations.append(
                    SafetyViolation(
                        'geofence',
                        f'Vehicle {vehicle_id} outside geofence',
                        (vehicle_id,),
                    )
                )

            velocity = state.velocity_local_enu
            if velocity is not None:
                speed = math.sqrt(
                    velocity.east ** 2
                    + velocity.north ** 2
                    + velocity.up ** 2
                )
                if speed > self.maximum_speed:
                    violations.append(
                        SafetyViolation(
                            'speed_limit',
                            f'Vehicle {vehicle_id} speed '
                            f'{speed:.2f} m/s exceeds limit',
                            (vehicle_id,),
                        )
                    )

        self._check_pair_distances(
            world_positions,
            violations,
            prefix='Vehicles',
        )

        return self._result(violations)

    def check_setpoints(self, states, setpoints):
        violations = []
        target_world_positions = {}

        if set(setpoints) != set(states):
            missing = set(states) - set(setpoints)
            violations.append(
                SafetyViolation(
                    'setpoint_missing',
                    'Missing setpoints for vehicles: '
                    + ', '.join(map(str, sorted(missing))),
                    tuple(sorted(missing)),
                )
            )

        for vehicle_id, setpoint in setpoints.items():
            if (
                vehicle_id not in states
                or vehicle_id not in self.vehicle_origins
            ):
                violations.append(
                    SafetyViolation(
                        'setpoint_vehicle_unknown',
                        f'Unknown setpoint vehicle {vehicle_id}',
                        (vehicle_id,),
                    )
                )
                continue

            target = setpoint.position_local_enu
            velocity = getattr(
                setpoint,
                'velocity_local_enu',
                None,
            )

            common_values = (
                target.east,
                target.north,
                target.up,
                setpoint.yaw_local_enu,
            )
            if not all(
                math.isfinite(value)
                for value in common_values
            ):
                violations.append(
                    SafetyViolation(
                        'setpoint_not_finite',
                        f'Vehicle {vehicle_id} setpoint not finite',
                        (vehicle_id,),
                    )
                )
                continue

            target_world = self._world_position(
                vehicle_id,
                target,
            )
            target_world_positions[vehicle_id] = target_world

            if velocity is not None:
                velocity_values = (
                    velocity.east,
                    velocity.north,
                    velocity.up,
                )
                if not all(
                    math.isfinite(value)
                    for value in velocity_values
                ):
                    violations.append(
                        SafetyViolation(
                            'velocity_setpoint_not_finite',
                            f'Vehicle {vehicle_id} velocity '
                            'setpoint not finite',
                            (vehicle_id,),
                        )
                    )
                    continue

                speed = math.sqrt(
                    velocity.east ** 2
                    + velocity.north ** 2
                    + velocity.up ** 2
                )
                if speed > self.maximum_speed:
                    violations.append(
                        SafetyViolation(
                            'velocity_setpoint_speed',
                            f'Vehicle {vehicle_id} velocity '
                            f'setpoint {speed:.2f} m/s exceeds limit',
                            (vehicle_id,),
                        )
                    )

                # In velocity-control mode the position field is only a
                # finite fallback/reference, not the command sent to PX4.
                # Do not apply position jump safety to velocity setpoints.
                continue

            if (
                target_world.up < 0.0
                or target_world.up > self.maximum_altitude
            ):
                violations.append(
                    SafetyViolation(
                        'setpoint_altitude',
                        f'Vehicle {vehicle_id} target altitude '
                        f'{target_world.up:.2f} m out of range',
                        (vehicle_id,),
                    )
                )

            if math.hypot(
                target_world.east,
                target_world.north,
            ) > self.geofence_radius:
                violations.append(
                    SafetyViolation(
                        'setpoint_geofence',
                        f'Vehicle {vehicle_id} target '
                        'outside geofence',
                        (vehicle_id,),
                    )
                )

            state = states[vehicle_id]
            if state.position_local_enu is not None:
                jump = vector_distance(
                    state.position_local_enu,
                    target,
                )
                if jump > self.maximum_setpoint_jump:
                    violations.append(
                        SafetyViolation(
                            'setpoint_jump',
                            f'Vehicle {vehicle_id} setpoint jump '
                            f'{jump:.2f} m exceeds limit',
                            (vehicle_id,),
                        )
                    )

        self._check_pair_distances(
            target_world_positions,
            violations,
            prefix='Target positions for vehicles',
        )

        return self._result(violations)

    def _check_pair_distances(
        self,
        positions,
        violations,
        prefix,
    ):
        vehicle_ids = sorted(positions)

        for first_index, first_id in enumerate(vehicle_ids):
            for second_id in vehicle_ids[first_index + 1:]:
                distance = vector_distance(
                    positions[first_id],
                    positions[second_id],
                )
                if distance < self.minimum_distance:
                    violations.append(
                        SafetyViolation(
                            'minimum_distance',
                            f'{prefix} {first_id} and {second_id} '
                            f'are {distance:.2f} m apart',
                            (first_id, second_id),
                        )
                    )

    def check(self, states, now_ns):
        return self.check_states(states, now_ns)
