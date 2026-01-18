from .gadget import Gadget, Message
from r0b0.utils.loaders import decode_msg
import logging

import pickle
from socketio import Namespace
from signal import pause
from threading import Thread
import asyncio
import numpy as np

import mouse

EVENTS = ["mouse_move", "mouse_button", "mouse_place"]

"""
Mouse move events:
- axis: 0 (x, horizontal), 1 (y, vertical)
- value
"""


class MouseMessage(Message):
    def __init__(self, **kwargs):
        Message.__init__(self, **kwargs)
        self.absolute = False


class Mouse(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        # self.on('mouse_move',
        #     handler=self.mouse_move_event,
        #     namespace=self.namespace)
        # self.on('mouse_button',
        #     handler=self.mouse_button_event,
        #     namespace=self.namespace)
        # self.on('mouse_place',
        #     handler=)
        self.handle_events(EVENTS)

        self.velocity = [0, 0, 0, 0]

    @decode_msg
    def mouse_move_event(self, data):
        print(data)
        msg = data["msg"]
        self.velocity[int(msg.axis)] = int(msg.value * 30)
        mouse.move(
            self.velocity[0], self.velocity[1], absolute=getattr(msg, "absolute", False)
        )
        pass

    @decode_msg
    def mouse_place_event(self, data):
        msg = data["msg"]
        mouse.move(msg.x, msg.y, absolute=True)

    @decode_msg
    def mouse_button_event(self, data):
        # logging.debug(data)
        msg = data["msg"]
        # logging.debug(msg.kwargs)
        mouse_func = getattr(mouse, msg.mouse_func)
        mouse_func(**msg.kwargs)
        pass
