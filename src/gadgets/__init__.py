from .gadget import Gadget
from .midi_controller import MIDIController
from .dxl_robot import DynamixelRobot
from .arduino import ArduinoGadget, ArduinoRobot
from .tape import Tape
from .phone import Phone
from .pygame_gadget import PyGameGadget, PyGameJoystick, PyGameKeys
from .mouse import Mouse

from src.utils.loaders import load_pickle, dump_pickle

# import logging
from src import logging
# logging = logging.getLogger(__name__)
# logging.basicConfig(
# #     # filename='example.log',
#     encoding='utf-8',
#     level=logging.INFO,
#     # level=logging.DEBUG,
#     )

