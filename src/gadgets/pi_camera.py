from .gadget import Gadget, Message
from src.config import TAPES_DIR
from src.utils.loaders import load_pickle
from src import logging, get_timestamp

import picamera
# native pi version
from picamera import PiCamera as _PiCamera
# import picamera.array
from picamera.array import PiRGBArray
import numpy as np
from time import sleep

shutter_speeds = [1/30, 1/250, 1/1000]
shutter_speed_idx = 0
RESOLUTION = (4056, 3040)
FRAMERATE = 15
ISO = 800
SHUTTER_BLINK_SLEEP = 0.5
SENSOR_MODE=3

class PiCamera(Gadget,_PiCamera):
    def __init__(self, config, **kwargs):
        Gadget.__init_(self, config, **kwargs)
        _PiCamera.__init__(self, sensor_mode=SENSOR_MODE)
        self.stream = PiRGBArray(self)
        # self.buttons = self.assign_buttons(config['buttons'])
        self.shutter_speed = int(shutter_speeds[shutter_speed_idx]*1000000)

        self.still_stats = True
        #camera.framerate = Fraction(1,1)
        self.iso = ISO
        self.framerate = FRAMERATE
        self.resolution = RESOLUTION

        self.on('shutter',
            handler=self.release_shutter)
        
        self.set_param = self.__dict__.update
        
    @load_pickle
    def release_shutter(self, msg, save_dir=TAPES_DIR):
        self.shutter_speed = msg.get(
            'shutter_speed',self.shutter_speed)
        logging.debug(f"Shutter released, ss={self.shutter_speed}")
        
        self.capture(str(TAPES_DIR / get_timestamp()))
    