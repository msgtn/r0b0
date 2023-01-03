from src.gadgets import MIDIController, Robot
from src.gadgets.server import start_server
from src.rigs.rig import Rig

from multiprocessing import Process
import signal

LOCALHOST = 'localhost'
SERVER_PORT = 8080
BLOSSOM_PORT = 9000
MIDI_PORT = 9002
BLOSSOM = 'blossomsingle'
# BLOSSOM = 'blossom'
OPZ = 'opz'

def cc2motor(data=None):
    if data is None: return {'event':'midi_cc'}
    # breakpoint()
    print(data)
    return {
        'event':'position',
        'value':(data.value/127.)*4096,
        'motor_id':data.control
    }

def main():
    # start server
    rig = Rig(hostname=LOCALHOST,port=SERVER_PORT,namespaces=['/opz','/blossomsingle'])
    rig.add_gadget(BLOSSOM)
    rig.add_gadget(OPZ)
    # breakpoint()
    # rig.add_server()
    rig.add_message(OPZ, BLOSSOM,cc2motor)
    # breakpoint()
    rig.power_on()        

    # input()
    # signal.signal(signal.SIGINT, rig.power_off)
    try:
        while True:
            continue
    except KeyboardInterrupt:
        rig.power_off('','')        
    
if __name__=="__main__":
    main()