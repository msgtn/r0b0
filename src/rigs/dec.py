from src.gadgets import MIDIController, Robot
from src.gadgets.rig import start_server
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