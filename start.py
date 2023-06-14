
from r0b0 import logging

# import argparse
# parser = argparse.ArgumentParser()
# parser.add_argument('--logging',type=str,default='warning')
# args = parser.parse_args()

logging.basicConfig(
    encoding='utf-8',
    # level=logging.DEBUG,
    # level=getattr(logging,args['logging'])
    level=logging.WARNING,
    )

from r0b0.rigs.rig import Rig
from r0b0.utils import loaders
from r0b0.cables import msg_funcs
# parser.add_argument(
#     '--log', default=sys.stdout, type=argparse.FileType('w'),
#     help='the file where the sum should be written')
# args.log.write('%s' % sum(args.integers))
# args.log.close()

import sys
from multiprocessing import Process
import signal
import numpy as np
# import logging
# logging.basicConfig(
#     # filename='example.log',
#     encoding='utf-8',
#     # level=logging.INFO,
#     level=logging.DEBUG,
#     )
from r0b0.config import LOCALHOST, SERVER_PORT

# LOCALHOST = 'localhost'
# SERVER_PORT = 8080

def main():
    config = loaders.load_rig(sys.argv[1])
    logging.debug(config)
    rig = Rig(
        hostname=config.get('hostname',LOCALHOST),
        port=config.get('port',SERVER_PORT),
        # TODO - get rid of this by adding to rig.add_gadget
        namespaces=[f'/{gadget}' for gadget in config['gadgets']],
        # namespaces=['/','/blossom'],
    )
    
    # add gadgets
    for gadget in config['gadgets']:
        rig.add_gadget(gadget)
    # connect cables
    for cable in config.get('cables',[]):
        rig.add_message(**cable)
    logging.debug('Powering rig')
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
    globals().update(**rig.gadgets)
    
    try:
        if rig.is_pygame_rig:
            rig.pygame_event_handler()
        else:
            print("Breakpoint")
            breakpoint()
    except KeyboardInterrupt:
        rig.power_off('','')        
    
