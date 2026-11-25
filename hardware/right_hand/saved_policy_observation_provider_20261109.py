"""Simulator observations at the training sensor sites; no hardware access."""
import numpy as np


def sensor_value(model, data, name, width):
    sensor = model.sensor(name)
    sid = sensor.id
    if int(model.sensor_dim[sid]) != width:
        raise ValueError(f'{name}: expected sensor dimension {width}')
    adr = int(model.sensor_adr[sid])
    value = np.asarray(data.sensordata[adr:adr+width], dtype=float).copy()
    if not np.isfinite(value).all():
        raise ValueError(f'{name}: non-finite observation')
    return value


def build_observation(builder, model, data, previous_goal, goal_quat,
                      position_source='training_sensor'):
    """Use the same pose sensor outputs as the original training environment.

    body_origin reproduces the earlier diagnostic error for comparison only.
    Neither choice defines a calibrated physical camera provider.
    """
    if position_source == 'training_sensor':
        palm = sensor_value(model, data, 'palm_position', 3)
        cube = sensor_value(model, data, 'cube_position', 3)
        orientation = sensor_value(model, data, 'cube_orientation', 4)
    elif position_source == 'body_origin':
        palm = data.xpos[model.body('palm').id]
        cube = data.xpos[model.body('cube').id]
        orientation = data.qpos[-4:]
    else:
        raise ValueError('Unknown observation provider: ' + position_source)
    return builder.build(data.qpos[:16], previous_goal, palm, cube, orientation, goal_quat)
