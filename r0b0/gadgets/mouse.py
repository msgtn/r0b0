from .gadget import Gadget, Message
from r0b0.utils.loaders import load_pickle
from r0b0 import logging

import pickle
from socketio import Namespace
from signal import pause
from threading import Thread
import asyncio
import numpy as np

import mouse
            
class Mouse(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self,config,**kwargs)
        self.on('mouse_move',
            handler=self.mouse_move_event,
            namespace=self.namespace)
        self.on('mouse_button',
            handler=self.mouse_button_event,
            namespace=self.namespace)
        self.velocity = [0,0,0,0]
        
    
    @load_pickle
    def mouse_move_event(self, data):
        msg = data['msg']
        # logging.debug(msg)
        self.velocity[int(msg.axis)] = int(msg.value*30)
        mouse.move(self.velocity[0],self.velocity[1],absolute=False)
        pass
    
    @load_pickle
    def mouse_button_event(self, data):
        logging.debug(data)
        msg = data['msg']
        logging.debug(msg.kwargs)
        mouse_func = getattr(
            mouse, msg.mouse_func)
        mouse_func(**msg.kwargs)
        pass
    