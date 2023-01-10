from src.rigs.rig import Rig
from src.utils.loaders import load_config
from src.messages import msg_funcs

import sys
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
    config = load_config(sys.argv[1])
    rig = Rig(
        hostname=config['hostname'],
        port=config['port'],
        # TODO - get rid of this
        namespaces=[f'/{gadget}' for gadget in config['gadgets']]
    )
    map(rig.add_gadget, config['gadgets'])
    # for message in config['messages']:
        # getattr(msg_funcs,message[''])
    map(lambda msg: rig.add_message(**msg), config['messages'])
    rig.power_on()

    try:
        breakpoint()
        # while True:
        #     continue
    except KeyboardInterrupt:
        rig.power_off('','')        

if __name__=="__main__":
    main()