from src.gadgets import MIDIController, Robot
from src.gadgets.server import start_server

from multiprocessing import Process
import signal
from rig import Rig

LOCALHOST = 'localhost'
SERVER_PORT = 8080
BLOSSOM_PORT = 9000
MIDI_PORT = 9002

def cc2motor(midi_cc):
    return {
        'type':'position',
        'position':(midi_cc.value/127.)*4096,
        'channel':midi_cc.channel
    }

def main():
    # start server
    rig = Rig()
    rig.add_gadget('blossom')
    rig.add_gadget('opz')
    rig.add_server(hostname=LOCALHOST,port=SERVER_PORT)
    rig.add_message('opz','blossom',cc2motor)
    rig.power_on()
    
    def handler(signum, frame):
        rig.power_off()
        
    # signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGINT, rig.power_off)
    while True:
        continue
    
if __name__=="__main__":
    main()