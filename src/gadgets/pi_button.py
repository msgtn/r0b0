from .gadget import Gadget, Message
from src.utils.loaders import load_pickle
from src import logging


import picamera
import picamera.array
import numpy as np
from time import sleep
from gpiozero import Button, LED
from signal import pause
from threading import Thread

import pickle
from socketio import Namespace

class PiButton(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        # self.buttons = self.assign_buttons(config['buttons'])
        
        pin_types = dict(
            buttons=Button,
            leds=LED,
        )
        self.__dict__.update({
            pin:self._assign_pins(
                config[pin],
                pin_type) for pin,pin_type in pin_types.items()
        })
        self._assign_buttons(self.__dict__.get('buttons',{}))
        self.pause_thread = Thread(
            target=pause)

        
    def _assign_pins(self, pin_dict, pin_type=Button):
        return {_name: pin_type(_pin) for _name,_pin in pin_dict.items()}
    
    def _assign_buttons(self, button_dict):
        for _name,_button in button_dict.items():
            _button.when_pressed = lambda: \
                self.emit(
                    event='pi_button',
                    data={'button':_name},
                )
              
    def start(self):
        Gadget.start(self)
        self.pause_thread.start()
        # pause()
