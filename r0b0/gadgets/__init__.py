from .gadget import Gadget
from .midi_controller import MIDIController
from .dxl_robot import DynamixelRobot
from .arduino import ArduinoGadget, ArduinoRobot
from .tape import Tape
from .page import Page, MobilePage
from .pygame_gadget import PyGameGadget, PyGameJoystick, PyGameKeys
from .mouse import Mouse

from r0b0.utils.loaders import load_msg, dump_msg

# import logging
from r0b0 import logging
# logging = logging.getLogger(__name__)
# logging.basicConfig(
# #     # filename='example.log',
#     encoding='utf-8',
#     level=logging.INFO,
#     # level=logging.DEBUG,
#     )

