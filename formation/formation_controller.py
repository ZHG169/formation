import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from formation.coordinate_convert import VectorENU
from formation.formation_shapes import get_shape
from formation.vehicle_interface import VehicleSetpoint
from formation.cpf_avoidance import CpfConfig, apply_cpf_to_setpoints

# 向量加法
def add_vectors(first, second):
    return VectorENU(
        east=first.east + second.east,
        north=first.north + second.north,
        up=first.up + second.up,
    )

#向量減法
def subtract_vectors(first, second):
    return VectorENU(
        east=first.east - second.east,
        north=first.north - second.north,
        up=first.up - second.up,
    )


def rotate_offset(offset, yaw_enu):
    cosine = math.cos(yaw_enu)
    sine = math.sin(yaw_enu)

    return VectorENU(
        east=cosine * offset.east - sine * offset.north,
        north=sine * offset.east + cosine * offset.north,
        up=offset.up,
    )


def vector_distance(first, second):
    return math.sqrt(
        (first.east - second.east) ** 2
        + (first.north - second.north) ** 2
        + (first.up - second.up) ** 2
    )


def normalize_yaw(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


def advance_reference(
    reference,
    velocity_enu,
    yaw_rate,
    dt_seconds,
):
    return FormationReference(
        position_world_enu=VectorENU(
            east=(
                reference.position_world_enu.east
                + velocity_enu.east * dt_seconds
            ),
            north=(
                reference.position_world_enu.north
                + velocity_enu.north * dt_seconds
            ),
            up=(
                reference.position_world_enu.up
                + velocity_enu.up * dt_seconds
            ),
        ),
        yaw_enu=normalize_yaw(
            reference.yaw_enu + yaw_rate * dt_seconds
        ),
    )


def set_reference_yaw(reference, yaw_enu):
    return FormationReference(
        position_world_enu=reference.position_world_enu,
        yaw_enu=normalize_yaw(yaw_enu),
    )


@dataclass(frozen=True)
class FormationReference:
    """Ground-generated reference in the common World ENU frame."""

    position_world_enu: VectorENU
    yaw_enu: float = 0.0


class FormationController(ABC):

    @abstractmethod
    def calculate(self, states, leader_id):
        raise NotImplementedError


class CentralizedFormationController(FormationController):

    def __init__(
        self,
        formation_type='triangle',
        spacing=2.0,
        vehicle_origins=None,
        formation_reference=None,
        control_period=0.01,
        cpf_config=None,
    ):
        self.formation_type = str(formation_type)
        self.spacing = float(spacing)
        self.vehicle_origins = dict(vehicle_origins or {})
        self.formation_reference = formation_reference
        self.control_period = float(control_period)
        self.cpf_config = cpf_config or CpfConfig()

        if self.formation_reference is None:
            raise ValueError('formation_reference is required')

        self.active_leader_id = None
        self.maximum_position_error = 0.0

    # 切換編隊形狀
    def set_formation(self, formation_type, spacing):
        get_shape(
            formation_type,
            spacing,
            self.vehicle_origins,
            next(iter(self.vehicle_origins)),
        )
        self.formation_type = str(formation_type)
        self.spacing = float(spacing)
    # 更新編隊中心
    def set_reference(self, reference):
        self.formation_reference = reference

    # 使用 leader command 或地面端 command 來推進 reference
    def apply_velocity_command(
        self,
        velocity_enu,
        yaw_rate,
        dt_seconds,
    ):
        self.formation_reference = advance_reference( #根據 command velocity 更新虛擬編隊中心
            self.formation_reference,
            velocity_enu,
            yaw_rate,
            dt_seconds,
        )

    # 更新編隊方向
    def set_reference_yaw(self, yaw_enu):
        self.formation_reference = set_reference_yaw(
            self.formation_reference,
            yaw_enu,
        )

    def calculate(self, states, leader_id):
        if leader_id not in states:
            raise ValueError(
                f'Leader {leader_id} state is unavailable'
            )

        missing_origins = set(states) - set(self.vehicle_origins)
        if missing_origins:
            raise ValueError(
                'Missing vehicle origins for IDs: '
                + ', '.join(map(str, sorted(missing_origins)))
            )

        for vehicle_id, state in states.items():
            if (
                not state.position_valid
                or state.position_local_enu is None
            ):
                raise ValueError(
                    f'Vehicle {vehicle_id} position invalid'
                )

        offsets = get_shape(
            name=self.formation_type,
            spacing=self.spacing,
            vehicle_ids=states,
            leader_id=leader_id,
        )

        commands = {}
        maximum_error = 0.0

        for vehicle_id, state in states.items():
            rotated_offset = rotate_offset(
                offsets[vehicle_id],
                self.formation_reference.yaw_enu,
            )
            target_world = add_vectors(
                self.formation_reference.position_world_enu,
                rotated_offset,
            )
            target_local = subtract_vectors(
                target_world,
                self.vehicle_origins[vehicle_id],
            )

            commands[vehicle_id] = VehicleSetpoint(
                position_local_enu=target_local,
                yaw_local_enu=self.formation_reference.yaw_enu,
            )

            current_world = add_vectors(
                self.vehicle_origins[vehicle_id],
                state.position_local_enu,
            )
            maximum_error = max(
                maximum_error,
                vector_distance(current_world, target_world),
            )

        self.active_leader_id = leader_id
        self.maximum_position_error = maximum_error

        return apply_cpf_to_setpoints(
            states=states,
            nominal_setpoints=commands,
            vehicle_origins=self.vehicle_origins,
            config=self.cpf_config,
            dt_seconds=self.control_period,
        )


class DistributedFormationController(FormationController):

    def calculate_local(
        self,
        own_state,
        neighbor_states,
        formation_offset,
    ):
        del neighbor_states, formation_offset

        if own_state.position_local_enu is None:
            raise ValueError('Own position is unavailable')

        return VehicleSetpoint(
            position_local_enu=own_state.position_local_enu,
            yaw_local_enu=own_state.yaw_local_enu,
        )

    def calculate(self, states, leader_id):
        del states, leader_id
        return {}
