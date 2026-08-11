from formation.coordinate_convert import VectorENU


def _validate_spacing(spacing):
    spacing = float(spacing)

    if spacing <= 0.0:
        raise ValueError('formation spacing must be positive')

    return spacing


def triangle(spacing):
    spacing = _validate_spacing(spacing)

    return [
        VectorENU(0.0, 0.0, 0.0),
        VectorENU(-spacing, spacing, 0.0),
        VectorENU(-spacing, -spacing, 0.0),
    ]


def line(spacing):
    spacing = _validate_spacing(spacing)

    return [
        VectorENU(0.0, 0.0, 0.0),
        VectorENU(-spacing, 0.0, 0.0),
        VectorENU(-2.0 * spacing, 0.0, 0.0),
    ]


def v_shape(spacing):
    spacing = _validate_spacing(spacing)

    return [
        VectorENU(0.0, 0.0, 0.0),
        VectorENU(-spacing, spacing, 0.0),
        VectorENU(-spacing, -spacing, 0.0),
    ]


def column(spacing):
    spacing = _validate_spacing(spacing)

    return [
        VectorENU(0.0, 0.0, 0.0),
        VectorENU(0.0, spacing, 0.0),
        VectorENU(0.0, 2.0 * spacing, 0.0),
    ]


def get_shape(name, spacing, vehicle_ids, leader_id):
    shape_functions = {
        'triangle': triangle,
        'line': line,
        'v_shape': v_shape,
        'column': column,
    }

    if name not in shape_functions:
        raise ValueError(f'Unknown formation: {name}')

    vehicle_ids = sorted(set(vehicle_ids))

    if leader_id not in vehicle_ids:
        raise ValueError(
            f'Leader {leader_id} is not in vehicle IDs'
        )

    offsets = shape_functions[name](spacing)

    if len(vehicle_ids) > len(offsets):
        raise ValueError(
            f'Formation {name} supports at most '
            f'{len(offsets)} vehicles'
        )

    ordered_ids = [
        leader_id,
        *(
            vehicle_id
            for vehicle_id in vehicle_ids
            if vehicle_id != leader_id
        ),
    ]

    return {
        vehicle_id: offsets[index]
        for index, vehicle_id in enumerate(ordered_ids)
    }
