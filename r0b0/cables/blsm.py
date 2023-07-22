from r0b0 import logging
import numpy as np
from r0b0.kinematics.blsm import device_motion2dxl_motor, \
    device_motion2dxl_motor320, \
    device_motion2arduino_motor

def motion2motor(data=None):
    if data is None: return {'event':'device_motion'}
    # logging.debug(f'motion2motor {data}')
    logging.debug(device_motion2dxl_motor(data))
    return {
        'event':'position',        # 'value': # the function that gives
        'value':device_motion2dxl_motor(data),
        'motor_id':[1,2,3,4],
        'absolute':True
    }
    
def motion2motor320(data=None):
    if data is None: return {'event':'device_motion'}
    # logging.debug(f'motion2motor {data}')
    logging.debug(device_motion2dxl_motor320(data))
    return {
        'event':'position',        # 'value': # the function that gives
        'value':device_motion2dxl_motor320(data),
        'motor_id':[1,2,3,4],
        'absolute':True
    }
    
def motion2arduino_motor(data=None):
    if data is None: return {'event':'device_motion'}
    # logging.debug(f'motion2motor {data}')
    return {
        'event':'position',        # 'value': # the function that gives
        'value':device_motion2arduino_motor(data),
        # changes these to string keys instead of int ids
        'motor_id':[10,6,5,9],
        'absolute':True
    }

def joy2rover(data=None):
    if data is None: return {'event':'joyaxismotion'}
    
    axis = data['axis']
    # logging.debug(data)
    if axis!=1: return {
        'event':'velocity',
        'motor_id':[],
        'value':[]
    }
    
    velocity = int(data['value']*1000)
    if np.abs(velocity)<100: velocity=0
    
    return {
        'event':'velocity',
        'motor_id':[7,8],
        'value':[velocity,-velocity],
        'absolute':False
    }
    
    
def motion2velocity(data=None):
    if data is None: return {'event':'device_motion'}
    # logging.debug(f'motion2motor {data}')
    return {
        'event':'position',        # 'value': # the function that gives
        'value':device_motion2dxl_motor(data),
        'motor_id':[1,2,3,4],
        # 'motor_kwargs':
            
        # 'absolute':True
    }

def joy2vel(data=None):
    if data is None: return {'event':'joyaxismotion'}
    motor_id,value = [],[]
    if data['axis']==1:
        motor_id = 1
        value = int(data['value']*2000)
        if np.abs(value)<200: value=0
    return {
        'event':'velocity',
        'motor_id':motor_id,
        'value':value,
        'absolute':True,
    }