import numpy as np
from src.kinematics.blossom import get_motor_pos
from src.utils.loaders import load_pickle, dump_pickle
import pickle
from src import logging

# def msg_func(func):
#     return

def msg_func(data=None, func=None, input_event=None, output_event=None):
    if data is None:
        return {'event':input_event}
    return {
        'event':output_event,
        **func(data)
    }

@load_pickle
def cc2motor(data=None):
    if data is None: return {'event':'midi_cc'}
    # data = pickle.loads(data['msg'])
    # print(data)
    return {
        'event':'position',
        'value':(data.value*4096)//127,
        'motor_id':(data.control)
    }
    
@load_pickle
def cc2ard(data=None):
    if data is None: return {'event':'midi_cc'}
    # data = pickle.loads(data['msg'])
    # print(data)
    return {
        'event':'position',
        'value':int(np.interp(data.value,[0,127],[10,160])),
        'motor_id':10
    }

@load_pickle
def note2motor(data=None):
    '''
    C4 = note value 60
    '''
    if data is None: return {'event':'midi_on'}
    # data = pickle.loads(data['msg'])
    return {
        'event':'position',
        'value':int(np.interp(
            data.note, [53,77], [0,4096])),
        'motor_id':(data.channel+1)
    }

@load_pickle
def motion2motor(data=None):
    if data is None: return {'event':'device_motion'}
    return {
        'event':'position',        # 'value': # the function that gives
        'value':get_motor_pos(data),
        'motor_id':[1,2,3,4]
    }
    
def motion2midi(data=None):
    if data is None: return {'event':'device_motion'}
    note = int(np.interp(np.rad2deg(data['x']),[0,180],[40,90]))
    return {
        'event':'midi',
        'type':'note_on',
        'note':note,
        'velocity':100,
        'channel':7,
    }

def button2cam(data=None):
    if data is None: return {'event':'pi_button'}
    return {
        'event':data['button'],
    }

def joy2midi(data=None):
    if data is None: return {'event':'joybutton'}
    event_dict = {
        'joybuttondown':'note_on',
        'joybuttonup':'note_off',
    }
    # print('joy2midi')
    # data = pickle.loads(data['msg'])
    # logging.debug(data)
    return {
        'event':'midi',
        'type':data['event_type'],
        'note':data['button']+40,
        'velocity':100,
        'channel':4
    }
    
def joy2motor(data=None):
    if data is None: return {'event':'joyaxismotion'}
    # if data['axis']!=3: return
    pos_val = data['value']
    # pos_val /= np.abs(pos_val)
    # pos_val = np.sin(pos_val*np.pi/2)
    logging.debug(pos_val)
    return {
        'event':'position',
        'value':int(np.interp(pos_val, [-1,1], [0,4096])),
        # 'motor_id':(data['axis']+1)
        # 'motor_id':(data['axis']+1)
        'motor_id':data['axis']
    }
    
# def dpad2shutter(data=None):
#     if data is None: return {'event':''}