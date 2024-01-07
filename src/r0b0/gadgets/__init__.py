# from .gadget import Gadget
# from .midi_controller import MIDIController
# from .dxl_robot import DynamixelRobot
# from .arduino import ArduinoGadget, ArduinoRobot
# from .tape import Tape
# from .page import Page, MobilePage
# from .pygame_gadget import PyGameGadget, PyGameJoystick, PyGameKeys
# from .mouse import Mouse
# from .rtc import RTCGadget
# from .camera import Camera
# from .microphone import Microphone
# from .chatbot import ChatBot

import os
import logging
from r0b0.utils import loaders
from importlib import import_module

GADGETS = {
    'gadget':['Gadget'],
    'midi_controller':['MIDIController'],
    'dxl_robot':['DynamixelRobot'],
    'arduino':['ArduinoGadget','ArduinoRobot'],
    'tape':['Tape'],
    'page':['Page','MobilePage'],
    'pygame_gadget':['PyGameGadget','PyGameJoystick','PyGameKeys'],
    'mouse':['Mouse'],
    'rtc':['RTCGadget'],
    'camera':['Camera'],
    'microphone':['Microphone'],
    'chatbot':['ChatBot'],
    'eink':['EInk'],
    'time_controller':['TimeController']
}
for pkg,mods in GADGETS.items():
    try:
        _pkg = import_module(f'r0b0.gadgets.{pkg}')
        globals().update({
            mod:getattr(_pkg,mod) for mod in mods
        })
    except:
        logging.warning(f'Error importing package {pkg}; its dependencies may not be installed')
    # for mod in mods:
    #     globals().update({})
        # import_module(f'{mod}',f'.{pkg}')
        # import_module(f'.{pkg}.{mod}','r0b0.gadgets')
    # import_module()

from r0b0.utils.loaders import decode_msg, encode_msg

# import logging

# logging = logging.getLogger(__name__)
# logging.basicConfig(
# #     # filename='example.log',
#     encoding='utf-8',
#     level=logging.INFO,
#     # level=logging.DEBUG,
#     )

# def from_config()

def from_config(gadget_yaml_path):
    """Create the gadget

    :param gadget_yaml_path: The path to the 
    :returns gadget: The gadget
    :raises Exception: The gadget class does not exist and cannot be created
    """
    config = loaders.load_yaml(gadget_yaml_path)
    gadget_cls = globals().get(config['type'], None)
    if gadget_cls is None:
        raise Exception(f"Gadget type {config['type']} does not exist")
    
    # If the name is not defined, used the filename
    if 'name' not in config:
        gadget_name = os.path.splitext(os.path.basename(gadget_yaml_path))[0]
        config.update({'name':gadget_name})
    # assert gadget_cls is not None, f"Gadget type {config['type']} does not exist"
    return gadget_cls(config)
