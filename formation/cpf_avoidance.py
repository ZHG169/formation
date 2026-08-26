import math
from dataclasses import dataclass

from formation.coordinate_convert import VectorENU
from formation.vehicle_interface import VehicleSetpoint


@dataclass(frozen=True)
class CpfConfig:
    enabled: bool = False
    attraction_gain: float = 0.8
    repulsion_gain: float = 1.2
    safe_distance: float = 1.5
    influence_distance: float = 4.0
    max_speed: float = 0.8
    output_velocity: bool = True
    fence_enabled: bool = False
    fence_world_x_min: float = -50.0
    fence_world_x_max: float = 50.0
    fence_world_y_min: float = -50.0
    fence_world_y_max: float = 50.0
    fence_height_min: float = 0.0
    fence_height_max: float = 20.0
    fence_brake_distance_m: float = 0.35


ZERO_VECTOR = VectorENU(0.0, 0.0, 0.0)



def clamp_value(value, lower, upper):
    return min(max(value, lower), upper)


def clamp_position_to_fence(position, config):
    if not config.fence_enabled:
        return position

    return VectorENU(
        east=clamp_value(
            position.east,
            config.fence_world_x_min,
            config.fence_world_x_max,
        ),
        north=clamp_value(
            position.north,
            config.fence_world_y_min,
            config.fence_world_y_max,
        ),
        up=clamp_value(
            position.up,
            config.fence_height_min,
            config.fence_height_max,
        ),
    )


def limit_velocity_near_fence(current, velocity, config):
    if not config.fence_enabled:
        return velocity
    if config.fence_brake_distance_m <= 1e-6:
        return velocity

    brake_distance = config.fence_brake_distance_m

    def cap(margin):
        ratio = clamp_value(margin / brake_distance, 0.0, 1.0)
        return config.max_speed * ratio

    east = velocity.east
    north = velocity.north

    if east > 0.0:
        east = min(east, cap(config.fence_world_x_max - current.east))
    elif east < 0.0:
        east = max(east, -cap(current.east - config.fence_world_x_min))

    if north > 0.0:
        north = min(north, cap(config.fence_world_y_max - current.north))
    elif north < 0.0:
        north = max(north, -cap(current.north - config.fence_world_y_min))

    return VectorENU(
        east=east,
        north=north,
        up=velocity.up,
    )

def add_vectors(first, second):
    return VectorENU(
        east=first.east + second.east,
        north=first.north + second.north,
        up=first.up + second.up,
    )


def subtract_vectors(first, second):
    return VectorENU(
        east=first.east - second.east,
        north=first.north - second.north,
        up=first.up - second.up,
    )


def vector_norm(vector):
    return math.sqrt(
        vector.east * vector.east
        + vector.north * vector.north
        + vector.up * vector.up
    )


def horizontal_norm(vector):
    return math.sqrt(
        vector.east * vector.east
        + vector.north * vector.north
    )


def scale_vector(vector, scale):
    return VectorENU(
        east=vector.east * scale,
        north=vector.north * scale,
        up=vector.up * scale,
    )


def limit_vector(vector, max_norm):
    norm = vector_norm(vector)

    if norm <= max_norm or norm <= 1e-6:
        return vector

    return scale_vector(vector, max_norm / norm)


def world_position(state, origin):
    return add_vectors(origin, state.position_local_enu)


def setpoint_world_position(setpoint, origin):
    return add_vectors(origin, setpoint.position_local_enu)


def valid_state(state):
    return bool(
        state.position_valid
        and state.position_local_enu is not None
    )


def repulsion_from_neighbors(
    vehicle_id,
    current_world,
    config,
):
    own_position = current_world[vehicle_id]
    repulsion = ZERO_VECTOR

    for other_id, other_position in current_world.items():
        if other_id == vehicle_id:
            continue

        away = subtract_vectors(own_position, other_position)
        distance = horizontal_norm(away)

        if distance >= config.influence_distance:
            continue

        if distance <= 1e-3:
            direction = VectorENU(1.0, 0.0, 0.0)
            distance = 1e-3
        else:
            direction = VectorENU(
                east=away.east / distance,
                north=away.north / distance,
                up=0.0,
            )

        base_strength = (
            (1.0 / distance)
            - (1.0 / config.influence_distance)
        ) / (distance * distance)

        strength = config.repulsion_gain * base_strength

        if distance < config.safe_distance:
            strength += config.repulsion_gain * (
                (config.safe_distance - distance)
                / config.safe_distance
            )

        repulsion = add_vectors(
            repulsion,
            scale_vector(direction, strength),
        )

    return repulsion


def apply_cpf_to_setpoints(
    states,
    nominal_setpoints,
    vehicle_origins,
    config,
    dt_seconds,
):
    if not config.enabled:
        return nominal_setpoints

    if dt_seconds <= 0.0:
        return nominal_setpoints

    current_world = {}
    for vehicle_id, state in states.items():
        if vehicle_id not in vehicle_origins:
            continue
        if not valid_state(state):
            continue
        current_world[vehicle_id] = world_position(
            state,
            vehicle_origins[vehicle_id],
        )

    safe_setpoints = {}

    for vehicle_id, nominal_setpoint in nominal_setpoints.items():
        if (
            vehicle_id not in current_world
            or vehicle_id not in vehicle_origins
        ):
            safe_setpoints[vehicle_id] = nominal_setpoint
            continue

        origin = vehicle_origins[vehicle_id]
        current = current_world[vehicle_id]
        target = setpoint_world_position(nominal_setpoint, origin)
        target = clamp_position_to_fence(target, config)

        attraction = scale_vector(
            subtract_vectors(target, current),
            config.attraction_gain,
        )
        attraction = limit_vector(
            attraction,
            config.max_speed,
        )

        repulsion = repulsion_from_neighbors(
            vehicle_id,
            current_world,
            config,
        )

        velocity = limit_vector(
            add_vectors(attraction, repulsion),
            config.max_speed,
        )
        velocity = limit_velocity_near_fence(
            current,
            velocity,
            config,
        )

        if config.output_velocity:
            safe_setpoints[vehicle_id] = VehicleSetpoint(
                position_local_enu=state.position_local_enu,
                yaw_local_enu=nominal_setpoint.yaw_local_enu,
                velocity_local_enu=velocity,
            )
            continue

        safe_world = add_vectors(
            current,
            scale_vector(velocity, dt_seconds),
        )
        safe_world = clamp_position_to_fence(safe_world, config)
        safe_local = subtract_vectors(safe_world, origin)

        safe_setpoints[vehicle_id] = VehicleSetpoint(
            position_local_enu=safe_local,
            yaw_local_enu=nominal_setpoint.yaw_local_enu,
        )

    return safe_setpoints
