from dataclasses import dataclass

from formation.coordinate_convert import VectorENU


@dataclass(frozen=True)
class FenceConfig:
    enabled: bool = False
    world_x_min: float = -50.0
    world_x_max: float = 50.0
    world_y_min: float = -50.0
    world_y_max: float = 50.0
    height_min: float = 0.0
    height_max: float = 20.0
    brake_distance_m: float = 0.35
    max_speed: float = 0.5


def clamp_value(value, lower, upper):
    return min(max(value, lower), upper)


def clamp_position_to_fence(position, config):
    if not config.enabled:
        return position

    return VectorENU(
        east=clamp_value(
            position.east,
            config.world_x_min,
            config.world_x_max,
        ),
        north=clamp_value(
            position.north,
            config.world_y_min,
            config.world_y_max,
        ),
        up=clamp_value(
            position.up,
            config.height_min,
            config.height_max,
        ),
    )


def limit_velocity_near_fence(current, velocity, config):
    if not config.enabled:
        return velocity
    if config.brake_distance_m <= 1e-6:
        return velocity

    brake_distance = config.brake_distance_m

    def cap(margin):
        ratio = clamp_value(margin / brake_distance, 0.0, 1.0)
        return config.max_speed * ratio

    east = velocity.east
    north = velocity.north
    up = velocity.up

    if east > 0.0:
        east = min(east, cap(config.world_x_max - current.east))
    elif east < 0.0:
        east = max(east, -cap(current.east - config.world_x_min))

    if north > 0.0:
        north = min(north, cap(config.world_y_max - current.north))
    elif north < 0.0:
        north = max(north, -cap(current.north - config.world_y_min))

    if up > 0.0:
        up = min(up, cap(config.height_max - current.up))
    elif up < 0.0:
        up = max(up, -cap(current.up - config.height_min))

    return VectorENU(
        east=east,
        north=north,
        up=up,
    )
