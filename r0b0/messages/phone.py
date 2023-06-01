from r0b0.kinematics.blossom import get_motor_pos

# @load_pickle
def motion2motor(data=None):
    if data is None: return {'event':'device_motion'}
    # logging.debug(f'motion2motor {data}')
    return {
        'event':'position',        # 'value': # the function that gives
        'value':get_motor_pos(data),
        'motor_id':[1,2,3,4]
    }
    