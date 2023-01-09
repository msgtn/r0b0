from src.gadgets import MIDIController, Robot
from src.gadgets.server import start_server
from src.rigs.rig import Rig

from multiprocessing import Process
import signal
import numpy as np

LOCALHOST = 'localhost'
SERVER_PORT = 8080
BLOSSOM_PORT = 9000
MIDI_PORT = 9002
BLOSSOM = 'blossomsingle'
BLOSSOM = 'blossom'
OPZ = 'opz'

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

def main():
    # start server
    rig = Rig(
        hostname=LOCALHOST,port=SERVER_PORT,
        namespaces=['/opz','/blossom'])
    rig.add_gadget(BLOSSOM)
    rig.add_gadget(OPZ)
    rig.add_message(OPZ, BLOSSOM,cc2motor)
    rig.power_on()        

    try:
        breakpoint()
        # while True:
        #     continue
    except KeyboardInterrupt:
        rig.power_off('','')        

if __name__=="__main__":
    main()