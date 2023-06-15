from r0b0 import logging
from r0b0.rigs.rig import Rig
from r0b0.utils import loaders
from r0b0.config import LOCALHOST, SERVER_PORT

import sys

def main():
    assert len(sys.argv)>1, 'No rig name from /config/rig/ provided, exiting'
    config = loaders.load_rig(sys.argv[1])
    logging.debug(config)
    rig = Rig(
        hostname=config.get('hostname',LOCALHOST),
        port=config.get('port',SERVER_PORT),
        # TODO - get rid of this by adding to rig.add_gadget
        namespaces=[f'/{gadget}' for gadget in config['gadgets']],
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

if __name__=="__main__":
    
    rig = main()
    # add gadgets from rig to namespace
    globals().update(**rig.gadgets)
    
    try:
        if rig.is_pygame_rig:
            rig.pygame_event_handler()
        else:
            print("CLI: ",end='')
            breakpoint()
    except KeyboardInterrupt:
        rig.power_off('','')        
    
