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
    # level=logging.INFO,
    level=logging.DEBUG,
    )
from src.config import LOCALHOST, SERVER_PORT

# LOCALHOST = 'localhost'
# SERVER_PORT = 8080

def main():
    config = loaders.load_rig(sys.argv[1])
    print(config)
    rig = Rig(
        hostname=config.get('hostname',LOCALHOST),
        port=config.get('port',SERVER_PORT),
        # TODO - get rid of this by adding to rig.add_gadget
        namespaces=[f'/{gadget}' for gadget in config['gadgets']],
        # namespaces=['/','/blossom'],
    )
    for gadget in config['gadgets']:
        rig.add_gadget(gadget)
    for msg in config.get('messages',[]):
        rig.add_message(**msg)
    print('powering')
    rig.power_on()
    return rig

def test_script(rig):
    tape_name = '20230131000558_device_motion'
    tape_name = '20230209011442_device_motion'
    tape_name = '20230210224815_device_motion'
    tape_name = '20230210225358_device_motion'
    tape_name = '20230210225448_device_motion'
    rig.on_load(
        {'tapeName':tape_name}
    )
    rig.on_play({
        'tape_name':tape_name
    })

if __name__=="__main__":
    rig = main()
    
    # test_script(rig)
    
    try:
        breakpoint()
    except KeyboardInterrupt:
        rig.power_off('','')        
    