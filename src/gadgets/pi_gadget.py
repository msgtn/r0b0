from .gadget import Gadget, Message
from src.utils.loaders import load_pickle
from src import logging


import numpy as np
from time import sleep
from gpiozero import Button, LED
from signal import pause
from threading import Thread
from functools import partial

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
        self.__dict__.update({
            pin:self.assign_pins(
                config[pin],
                pin_type) for pin,pin_type in pin_types.items()
        })
        self.assign_buttons(
            self.__dict__.get('buttons',{}))
        logging.debug(self.__dict__.get('buttons',"No buttons"))
        self.pause_thread = Thread(
            target=pause)

        
    def assign_pins(self, pin_dict, pin_type=Button):
        return {_name: pin_type(_pin) for _name,_pin in pin_dict.items()}
    
    def _emit_button(self,button_name,event='pi_button'):
        return lambda : self.emit(
            event=event,
            data={'button':button_name},
            namespace=self.namespace)

    def assign_buttons(self, button_dict):
        logging.debug(button_dict)
        for button_name,button in button_dict.items():
            button.when_pressed = self._emit_button(button_name)
              
    def start(self):
        Gadget.start(self)
        self.pause_thread.start()
        # pause()


class PiButton(PiGadget):
    def __init__(self, **kwargs):
        PiGadget.__init__(self,**kwargs)