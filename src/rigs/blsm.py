from src.gadgets import MIDIController, Robot
from src.gadgets.server import start_server
from src.rigs.rig import Rig
from src.kinematics.blossom import get_motor_pos

from multiprocessing import Process
import signal
import numpy as np

LOCALHOST = 'localhost'
SERVER_PORT = 8080
BLOSSOM_PORT = 9000
MIDI_PORT = 9002
BLOSSOM = 'blossomsingle'
BLOSSOM = 'blossom'
# BLOSSOM = 'testgadget'
PHONE = 'phone'

def motion2motor(data=None):
    if data is None: return {'event':'device_motion'}
    # inverse kinematics
    # motor_values = inv_kin(data)
    # print(get_motor_pos(data))
    # messages need to handle lists of values
    # return [{
    #     'event':'position',
    #     'value':motor_value,
    #     'motor_id':motor_id
    #     } for motor_value,motor_id in zip(motor_values,range(1,5))]
    return {
        'event':'position',
        # 'value': # the function that gives
        'value':get_motor_pos(data),
        'motor_id':[1,2,3,4]
    }
    
def main():
    # start server
    rig = Rig(
        hostname=LOCALHOST,
        port=SERVER_PORT,
        # namespaces=['/phone','/blossomsingle']
        namespaces=['/','/blossom']
    )
    rig.add_gadget(BLOSSOM)
    rig.add_gadget(PHONE)
    rig.add_message(PHONE, BLOSSOM, motion2motor)
    rig.power_on()        

    try:
        breakpoint()
        # while True:
        #     continue
    except KeyboardInterrupt:
        rig.power_off('','')        
    
if __name__=="__main__":
    main()