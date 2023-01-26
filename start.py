from src.rigs.rig import Rig
from src.utils import loaders
from src.messages import msg_funcs

import sys
from multiprocessing import Process
import signal
import numpy as np
import logging
logging.basicConfig(
    # filename='example.log',
    encoding='utf-8',
    level=logging.INFO,
    )

LOCALHOST = 'localhost'
SERVER_PORT = 8080

def main():
    config = loaders.load_rig(sys.argv[1])
    rig = Rig(
        hostname=config.get('hostname',LOCALHOST),
        port=config.get('port',SERVER_PORT),
        # TODO - get rid of this by adding to rig.add_gadget
        namespaces=[f'/{gadget}' for gadget in config['gadgets']]
    )
    for gadget in config['gadgets']:
        rig.add_gadget(gadget)
    for msg in config.get('messages',[]):
        rig.add_message(**msg)
    rig.power_on()

    try:
        breakpoint()
    except KeyboardInterrupt:
        rig.power_off('','')        

if __name__=="__main__":
    main()