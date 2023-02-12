import numpy as np
#from src.kinematics.blossom import get_motor_pos
import pickle

def cc2motor(data=None):
    if data is None: return {'event':'midi_cc'}
    print(data)
    return {
        'event':'position',
        'value':(data.value*4096)//127,
        'motor_id':data.control
    }
    
def note2motor(data=None):
    '''
    C4 = note value 60
    '''
    if data is None: return {'event':'midi_on'}
    data = pickle.loads(data['msg'])
    # print(data)
    return {
        'event':'position',
        'value':int(np.interp(data.note, [53,77], [0,4096])),
        'motor_id':(data.channel+1)
    }

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
        'event':'shutter',
    }
