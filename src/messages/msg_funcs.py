
def cc2motor(data=None):
    if data is None: return {'event':'midi_cc'}
    return {
        'event':'position',
        'value':(data.value*4096)//127,
        'motor_id':data.control
    }
    
def note2motor(data=None):
    if data is None: return {'event':'midi_on'}
    return {
        'event':'position',
        'value':int(np.interp(data.value, [30, 60], [0,4096])),
        'motor_id':data.control
    }

def motion2motor(data=None):
    if data is None: return {'event':'device_motion'}
    return {
        'event':'position',        # 'value': # the function that gives
        'value':get_motor_pos(data),
        'motor_id':[1,2,3,4]
    }