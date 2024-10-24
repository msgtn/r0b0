from .gadget import Gadget, Message
import logging
from r0b0.utils.loaders import decode_msg
from r0b0.config import TAPES_DIR

import numpy as np
from time import sleep
from signal import pause
from threading import Thread
from functools import partial

from picamera2 import Picamera2
from gpiozero import Button, LED

import pickle
from socketio import Namespace
import copy


class PiGadget(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)

        pin_types = dict(
            buttons=Button,
            leds=LED,
        )
        self.__dict__.update(
            {
                pin: self.assign_pins(config[pin], pin_type)
                for pin, pin_type in pin_types.items()
            }
        )
        self.assign_buttons(self.__dict__.get("buttons", {}))
        logging.debug(self.__dict__.get("buttons", "No buttons"))
        self.pause_thread = Thread(target=pause)

    def assign_pins(self, pin_dict, pin_type=Button):
        return {_name: pin_type(_pin) for _name, _pin in pin_dict.items()}

    def _emit_button(self, button_name, event="pi_button"):
        return lambda: self.emit(
            event=event, data={"button": button_name}, namespace=self.namespace
        )

    def assign_buttons(self, button_dict):
        logging.debug(button_dict)
        for button_name, button in button_dict.items():
            button.when_pressed = self._emit_button(button_name)

    def start(self):
        Gadget.start(self)
        self.pause_thread.start()
        # pause()


class PiButton(PiGadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)

        pin_types = dict(
            buttons=Button,
            leds=LED,
        )
        self.__dict__.update(
            {
                pin: self.assign_pins(config[pin], pin_type)
                for pin, pin_type in pin_types.items()
            }
        )
        self.assign_buttons(self.__dict__.get("buttons", {}))
        logging.debug(self.__dict__.get("buttons", "No buttons"))
        self.pause_thread = Thread(target=pause)

    def _assign_pins(self, pin_dict, pin_type=Button):
        return {_name: pin_type(_pin) for _name, _pin in pin_dict.items()}

    def _emit_button(self, button_name, event="pi_button"):
        return lambda: self.emit(
            event=event, data={"button": button_name}, namespace=self.namespace
        )

    def assign_buttons(self, button_dict):
        # _emit_button = lambda button_name: \
        #     self.emit(
        #         event='pi_button',
        #         data={'button':button_name}
        #     )
        logging.debug(button_dict)
        for button_name, button in button_dict.items():
            # _button.when_pressed = _emit_button(_name)
            button.when_pressed = self._emit_button(button_name)

    def start(self):
        Gadget.start(self)
        self.pause_thread.start()
        # pause()


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
        logging.debug(f"Shutter released")
        # TODO - split off into separate subfolders
        # instead of one large folder
        self.capture_file(str(TAPES_DIR / f"picam_{get_file_number(TAPES_DIR)}.jpg"))

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
