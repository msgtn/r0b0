from src.rigs.rig import Rig
from src.utils import loaders
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

def main():
    config = loaders.load_rig(sys.argv[1])
    rig = Rig(
        hostname=config.get('hostname',LOCALHOST),
        port=config.get('port',SERVER_PORT),
        # TODO - get rid of this by adding to rig.add_gadget
        namespaces=[f'/{gadget}' for gadget in config['gadgets']]
    )
    # map(rig.add_gadget, config['gadgets'])
    for gadget in config['gadgets']:
        rig.add_gadget(gadget)
    # breakpoint()
    for msg in config['messages']:
        rig.add_message(**msg)
    # map(lambda msg: rig.add_message(**msg), config['messages'])
    rig.power_on()

    try:
        breakpoint()
    except KeyboardInterrupt:
        rig.power_off('','')        

if __name__=="__main__":
    main()