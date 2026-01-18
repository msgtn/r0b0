from .gadget import Gadget, Message
from r0b0.config import TAPES_DIR
from r0b0.utils.loaders import decode_msg
from r0b0 import logging, get_timestamp
import os

from picamera2 import Picamera2
import numpy as np
from time import sleep
from functools import partial
import glob
import subprocess

shutter_speeds = [1 / 30, 1 / 250, 1 / 1000]
shutter_speed_idx = 0
RESOLUTION = (4056, 3040)
FRAMERATE = 15
ISO = 800
SHUTTER_BLINK_SLEEP = 0.5
SENSOR_MODE = 3

get_file_number = lambda save_dir: len(glob.glob(str(save_dir / "*")))


class PiCamera(Gadget, Picamera2):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        # _PiCamera.__init__(self, sensor_mode=SENSOR_MODE)
        Picamera2.__init__(self)
        Picamera2.create_still_configuration(self)
        # self.still_configuration.size = (4056,3040)
        self.still_configuration.size = (2028, 1520)
        Picamera2.start(self, "still")

        try:
            os.remove(str(TAPES_DIR / "testshot.jpg"))
        except:
            pass
        self.capture_file(str(TAPES_DIR / "testshot.jpg"))

        self.on("shutter", handler=self.release_shutter, namespace=self.namespace)
        try:
            logging.debug(self.camera_controls)
        except:
            pass

        self.on("d_down", handler=self.shutter15, namespace=self.namespace)
        self.on("d_right", handler=self.shutter60, namespace=self.namespace)
        self.on("d_up", handler=self.shutter250, namespace=self.namespace)
        self.on("d_left", handler=self.shutter1000, namespace=self.namespace)

        self.on(
            "set_shutter_speed",
            handler=self.on_set_shutter_speed,
            namespace=self.namespace,
        )
        self.set_param = self.__dict__.update
        self.start = lambda: Gadget.start(self)

    @decode_msg
    def release_shutter(self, msg, save_dir=TAPES_DIR, **kwargs):
        subprocess.call(["raspi-gpio", "set", "47", "dh"])
        logging.debug(f"Shutter released")
        # TODO - split off into separate subfolders
        # instead of one large folder
        self.capture_file(str(TAPES_DIR / f"picam_{get_file_number(TAPES_DIR)}.jpg"))

    @decode_msg
    def set_shutter(self, shutter_speed):
        """
        Set the shutter speed of the camera.

        :param shutter_speed: Shutter speed in milliseconds, i.e. 1e6 / {15, 60, 250}.
        """
        logging.debug(f"Shutter speed: 1/{int(1e6 / shutter_speed)}")
        self.set_controls({"ExposureTime": shutter_speed})

    @decode_msg
    def shutter15(
        self,
        msg,
    ):
        logging.debug("Shutter: 1/15")
        self.set_controls({"ExposureTime": int(10e5 / 15)})
        return

    @decode_msg
    def shutter60(
        self,
        msg,
    ):
        logging.debug("Shutter: 1/60")
        self.set_controls({"ExposureTime": int(10e5 / 60)})
        return

    @decode_msg
    def shutter250(
        self,
        msg,
    ):
        logging.debug("Shutter: 1/250")
        self.set_controls({"ExposureTime": int(10e5 / 250)})
        return

    @decode_msg
    def shutter1000(
        self,
        msg,
    ):
        logging.debug("Shutter: 1/1000")
        self.set_controls({"ExposureTime": int(10e5 / 1000)})
        return

    @decode_msg
    # def on_set_shutter_speed(self, shutter_speed: float, **kwargs):
    def on_set_shutter_speed(self, data, **kwargs):
        msg = data["msg"]
        ss = int(msg.shutter_speed)
        logging.debug(f"Shutter: {ss}")
        self.set_controls({"ExposureTime": ss})
