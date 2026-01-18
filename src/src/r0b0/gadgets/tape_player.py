from .gadget import Gadget, Message
from r0b0.utils.loaders import decode_msg
import logging

import pickle
from socketio import Namespace
from signal import pause
from threading import Thread
import asyncio
import numpy as np

EVENTS = ["prompt"]


class Mouse(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.handle_events(EVENTS)

    @decode_msg
    def load_event(self, data):
        msg = data["msg"]
        self.emit(event="load", data={"event": "load", "tape_name": msg.tape_name})

    @decode_msg
    def play_event(self, data):
        msg = data["msg"]
        self.emit(event="play", data={"event": "play", "tape_name": msg.tape_name})
