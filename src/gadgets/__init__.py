from .gadget import Gadget
from .midi_controller import MIDIController
from .robot import Robot
from .arduinobot import ArduinoBot
from .tape import Tape
from .phone import Phone
from src.utils.loaders import load_pickle, dump_pickle
import logging
logging = logging.getLogger(__name__)
# logging.basicConfig(
#     # filename='example.log',
#     encoding='utf-8',
#     # level=logging.INFO,
#     level=logging.DEBUG,
#     )

