from r0b0.rigs.rig import Rig
from r0b0.utils import loaders
from r0b0.cables import msg_funcs
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0 import logging, \
    gadgets as gadget_shelf
from functools import partial

import sys
import argparse

def create_gadget(gadget_name):
    config = loaders.load_gadget(gadget_name)
    gadget_cls = getattr(
        gadget_shelf, config['type'], None)
    assert gadget_cls is not None, f"Gadget type {config['type']} does not exist"
    return gadget_cls(config)

def main(rig_config):
    config = loaders.load_rig(rig_config)
    logging.debug(config)
    rig = Rig(
        hostname=config.get('hostname',LOCALHOST),
        port=config.get('port',SERVER_PORT),
        # TODO - get rid of this by adding to rig.add_gadget
        # namespaces=[f'/{gadget}' for gadget in config['gadgets']],
    )
    # breakpoint()
    # add gadgets
    gadgets = {}
    for gadget_name in config['gadgets']:
        gadget_obj = create_gadget(gadget_name)
        rig.add_gadget(gadget_obj)
        gadgets.update({gadget_name:gadget_obj})
    # connect cables
    for cable in config.get('cables',[]):
        rig.add_cable(**cable)
    logging.debug('Powering rig')
    rig.power_on()
    return rig, gadgets

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
    
    rig,gadgets = main(rig_config)
    globals().update(**gadgets)
        
    try:
        if rig.is_pygame_rig:
            rig.pygame_event_handler()
        else:
            breakpoint(header="Starting command-line interface, (Ctrl+D) to exit")
    except KeyboardInterrupt:
        rig.power_off('','')        
    
