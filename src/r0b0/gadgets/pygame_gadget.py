from .gadget import Gadget, Message
from r0b0.utils.loaders import decode_msg
import logging

import pickle
from socketio import Namespace
from signal import pause
from threading import Thread
import asyncio
import numpy as np

import pygame
from pygame import (
    joystick as pgJoystick,
    event as pgEvent,
    display as pgDisplay,
    mouse as pgMouse,
)

# from pygame.joystick import Joystick as pgJoystick
# pgEvent.init()
pygame.init()
# pgDisplay.init()
pgJoystick.init()

_pg_gadgets = [
    "Joystick",
    "TextInput",
]

_py_events = [
    "QUIT",
    # joysticks
    # {'event': 'JoyAxisMotion', 'data': {'joy': 0, 'instance_id': 0, 'axis': 2, 'value': 0.08233893856624043}, 'namespace': '/x3d'}
    "JOYAXISMOTION",
    "JOYBALLMOTION",
    "JOYBUTTONDOWN",
    # {'event': 'JoyButtonDown', 'data': {'joy': 0, 'instance_id': 0, 'button': 11}, 'namespace': '/x3d'}
    "JOYBUTTONUP",
    "JOYHATMOTION",
]
EVENT_TABLE = {getattr(pygame, _py_event): _py_event for _py_event in _py_events}

JOY_AXIS_THRESH = 0.01
# types of pygame gadgets
# Joystick,


class PyGameGadget(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.config = config
        self.pygame_name = ""
        # self.joystick = pgJoystick.Joystick.__init__(self,
        # id=self.config.get('id',0))


class PyGameJoystick(PyGameGadget):
    def __init__(self, config, **kwargs):
        PyGameGadget.__init__(self, config, **kwargs)
        joy_id = self.config.get("id", 0)
        self.pygame_name = f"joy_{joy_id}"
        if pgJoystick.get_count():
            self.joystick = pgJoystick.Joystick(joy_id)

    def emit(self, event, data, **kwargs):
        event_dict = {
            "joybuttondown": "button_down",
            "joybuttonup": "button_up",
        }
        # event_dict.setdefault()
        # event = event_dict.get(event,event)

        # LogiTech Extreme3DPro tables
        # roll, pitch, yaw, throttle(?) = [0,1,2,3]
        # all are in range [-1,1]
        # roll = [left,right] = [-1,1]
        # pitch = [forward, backward] = [-1,1]
        # yaw = [cw, ccw] = [-1,1]
        # throttle = [+,-] = [-1,1] (unintuitive)

        if "axis" in event:
            # don't send axis events below threshold
            if np.abs(data["value"]) < JOY_AXIS_THRESH:
                return
        elif "button" in event:
            data.update(dict(event_type="joybutton", button_press=event_dict[event]))
            event = "joybutton"

        Gadget.emit(self, event, data, **kwargs)
        logging.debug(event)
        pass


class PyGameKeys(PyGameGadget):
    def __init__(self, config, **kwargs):
        PyGameGadget.__init__(self, config, **kwargs)
        self.pygame_name = "keys"
        self.on("pygamekeys", handler=self.key_event, namespace=self.namespace)
        self.on("pygamemouse", handler=self.mouse_event, namespace=self.namespace)

    @decode_msg
    def mouse_event(self, data):
        msg = data["msg"]
        logging.debug("mouse event")
        logging.debug(msg.axis)
        logging.debug(msg.value)
        # logging.debug(msg.data)
        # logging.debug(msg.value)
        # pgMouse.set_pos(msg['pos'])
        cur_pos = pgMouse.get_pos()
        logging.debug(cur_pos)
        tgt_pos = list(pgMouse.get_pos())
        if int(msg.axis) < len(tgt_pos):
            tgt_pos[int(msg.axis)] += int(msg.value * 10)
        pgMouse.set_pos(tgt_pos)
        pass

    @decode_msg
    def key_event(self):
        pass


class PyGameMouse(PyGameGadget):
    def __init__(self, config, **kwargs):
        PyGameGadget.__init__(self, config, **kwargs)
        self.pygame_name = "mouse"
        self.on("pygamemouse", handler=self.mouse_event, namespace=self.namespace)

    @decode_msg
    def mouse_event(self):
        pass
