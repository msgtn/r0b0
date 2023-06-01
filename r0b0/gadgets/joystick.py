from .gadget import Gadget, Message
from r0b0.utils.loaders import load_pickle
from r0b0 import logging

import pickle
from socketio import Namespace
from signal import pause
from threading import Thread
import asyncio
import numpy as np

import pygame
from pygame import joystick as pgJoystick, \
    event as pgEvent, \
    display as pgDisplay
# from pygame.joystick import Joystick as pgJoystick
# pgEvent.init()
# pygame.init()
# pgDisplay.init()
# pgJoystick.init()

_py_events = [
    'QUIT',
    # joysticks
    # {'event': 'JoyAxisMotion', 'data': {'joy': 0, 'instance_id': 0, 'axis': 2, 'value': 0.08233893856624043}, 'namespace': '/x3d'}
    'JOYAXISMOTION',
    'JOYBALLMOTION',
    'JOYBUTTONDOWN',
    # {'event': 'JoyButtonDown', 'data': {'joy': 0, 'instance_id': 0, 'button': 11}, 'namespace': '/x3d'}
    'JOYBUTTONUP',
    'JOYHATMOTION'
]
EVENT_TABLE = {
    getattr(pygame, _py_event):_py_event for _py_event in _py_events
}
MUTE_EVENTS = [
    'clock'
]
MIDO_ARGS = [
    'type','channel',
    'note','velocity',
    'value',
    'program',
    'pitch',
    'control'
]

class Joystick(Gadget):
    def __init__(self,config,**kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.config = config
        # self.joystick = pgJoystick.Joystick.__init__(self,
            # id=self.config.get('id',0))
        if pgJoystick.get_count():
            self.joystick = pgJoystick.Joystick(
                self.config.get('id',0)
            )
        
        self.event_thread = Thread(
            target=self.pgEvent_thread,
        )
    
    def pygame_event_loop(self,loop, event_queue):
        while True:
            event = pgEvent.wait()
            asyncio.run_coroutine_threadsafe(event_queue.put(event), loop=loop)

        
    def pgEvent_thread(self):
    # async def pgEvent_thread(self):
    # async def pgEvent_thread(self, event_queue):
    # async def pgEvent_thread(self):
        
        # pygame.init()
        
        
        while True:
            # event = await event_queue.get()
            # print(event)
        # asyncio.get_event_loop().stop()
            for event in pgEvent.get():
                # print(event.type)
                pass
        pass

    def start(self):
        Gadget.start(self)
        # self.event_thread.start()
        # print(self.joystick.)
        # time.sleep(5)
        while True:
            
            for event in pgEvent.get():
                _event_name = pgEvent.event_name(event.type)
                emit_dict =  dict(
                    event=_event_name,
                    data=event.__dict__,
                    namespace=self.namespace,
                )
                # print(emit_dict)
                try:
                    self.emit(**emit_dict)
                    if 'axis' not in _event_name.lower():
                        print(emit_dict)
                    elif np.abs(event.__dict__['value'])>0.1:
                        # roll, pitch, yaw, throttle(?) = [0,1,2,3]
                        # all are in range [-1,1]
                        # roll = [left,right] = [-1,1]
                        # pitch = [forward, backward] = [-1,1]
                        # yaw = [cw, ccw] = [-1,1]
                        # throttle = [+,-] = [-1,1] (unintuitive)
                        
                        # if event.__dict__['axis']==0:
                            # print(emit_dict)
                        print(emit_dict)
                except:
                    pass