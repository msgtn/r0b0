from .gadget import Gadget, Message
from r0b0.config import TAPES_DIR
from r0b0.utils.loaders import decode_msg
from r0b0 import logging
, get_timestamp
import os

import picamera

# native pi version
from picamera import PiCamera as _PiCamera
from picamera2 import Picamera2, Preview

# import picamera.array
from picamera.array import PiRGBArray
import numpy as np
from time import sleep
import glob

shutter_speeds = [1 / 30, 1 / 250, 1 / 1000]
shutter_speed_idx = 0
RESOLUTION = (4056, 3040)
FRAMERATE = 15
ISO = 800
SHUTTER_BLINK_SLEEP = 0.5
SENSOR_MODE = 3

# def get_file_number(dir):
#     return

get_file_number = lambda save_dir: len(glob.glob(str(save_dir / "*")))


# class PiCamera(Gadget,_PiCamera):
class PiScreen(Gadget, Picamera2):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        # _PiCamera.__init__(self, sensor_mode=SENSOR_MODE)
        Picamera2.__init__(self)
        Picamera2.start(self)
        try:
            os.remove(str(TAPES_DIR / "testshot.jpg"))
        except:
            pass
        self.capture_file(str(TAPES_DIR / "testshot.jpg"))

        self.on(
            "shutter",
            handler=self.release_shutter,
            # handler=lambda : \
            #     self.capture_file(
            #         str(TAPES_DIR / f"{get_timestamp()}_picam.jpg")
            #     )
            namespace=self.namespace,
        )

        self.set_param = self.__dict__.update
        self.start = lambda: Gadget.start(self)

    @decode_msg
    def release_shutter(self, msg, save_dir=TAPES_DIR):
        # self.shutter_speed = msg.get(
        #     'shutter_speed')
        logging.debug(f"Shutter released")
        # logging.debug(f"Shutter released, ss={self.shutter_speed}")
        self.capture_file(
            # str(TAPES_DIR / f"{get_timestamp()}_picam.jpg")
            str(TAPES_DIR / f"picam_{get_file_number(TAPES_DIR)}.jpg")
        )

        # self.camera.capture(str(TAPES_DIR / get_timestamp()))

    def start(self):
        Gadget.start(self)
        return

        with _PiCamera(sensor_mode=SENSOR_MODE) as camera:
            with PiRGBArray(camera) as stream:
                camera.iso = ISO
                shutter_speed = shutter_speeds[0]
                camera.shutter_speed = int(shutter_speeds[shutter_speed_idx] * 1000000)
                # camera.awb_mode = 'auto'
                # camera.framerate=90
                logging.debug("Initializing Pi Camera")
                # sleep(1)
                # camera.exposure_mode = 'off'
                camera.still_stats = True
                # camera.framerate = Fraction(1,1)
                camera.framerate = FRAMERATE
                camera.resolution = RESOLUTION
                logging.debug("capturing wth camera")
                camera.capture(str(TAPES_DIR / "testshot.jpg"))
