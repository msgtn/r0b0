
from r0b0 import logging


logging.basicConfig(
    encoding='utf-8',
    level=logging.WARNING,
    )

from r0b0.rigs.rig import Rig
from r0b0.utils import loaders

import sys


from r0b0.config import LOCALHOST, SERVER_PORT

def main():
    config = loaders.load_rig(sys.argv[1]) # saw we took out argparse--does load_rig handle null arg? may want to check/throw/log something if no arg
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

def test_script(rig): # MATT - keep this here? it's not called. also, is there a reason to keep redefining tape_name?
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
    
    globals().update(**rig.gadgets)
    
    try:
        if rig.is_pygame_rig:
            rig.pygame_event_handler()
        else:
            print("Breakpoint") # MATT - should this be a logger call instead?
            breakpoint()
    except KeyboardInterrupt:
        rig.power_off('','')        
    
