from r0b0.kinematics.blsm import device_motion2dxl_motor

# @load_pickle
def motion2motor(data=None):
    if data is None: return {'event':'device_motion'}
    # logging.debug(f'motion2motor {data}')
    return {
        'event':'position',        # 'value': # the function that gives
        'value':device_motion2dxl_motor(data),
        'motor_id':[1,2,3,4]
    }
