from r0b0.rigs.rig import Rig
from r0b0.utils import loaders
from r0b0.cables import msg_funcs
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0 import logging
from functools import partial

import sys
import argparse

def main(rig_config):
    config = loaders.load_rig(rig_config)
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
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--logging',type=str,default='warning')
    parser.add_argument('--config',type=str,default=None)
    args = parser.parse_args()

    logging.basicConfig(
        encoding='utf-8',
        level=getattr(logging,args.logging.upper())
    )
    
    rig_config = args.config if args.config is not None else sys.argv[1]
    assert rig_config is not None, "No rig config provided, either as sys.argv[1] or with --config"
    
    rig = main(rig_config)
    globals().update(**rig.gadgets)
    
    tape = rig.on_load({
        'tape_name':'20230720201152_device_motion'})
    play_tape = partial(rig.on_play, data={
        'tape_name':'20230720201152_device_motion'})
    
    try:
        if rig.is_pygame_rig:
            rig.pygame_event_handler()
        else:
            print("Breakpoint")
            breakpoint()
    except KeyboardInterrupt:
        rig.power_off('','')        
    
