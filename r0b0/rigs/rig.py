# from r0b0 import gadgets
# import r0b0.gadgets
from functools import partial
from collections import defaultdict
from r0b0 import gadgets as gadget_shelf, \
    messages as r0b0_msgs
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.utils import loaders
# from r0b0.rigs import logging
# from r0b0.gadgets.rig import start_server
from r0b0.rigs.server import Host
# from r0b0.messages import msg_funcs
from multiprocessing import Process
import pickle
from  r0b0 import logging
import time

# logging = logging.getLogger(__name__)


import pygame
from pygame import joystick as pgJoystick, \
    event as pgEvent, \
    display as pgDisplay, \
    time as pgTime
# from pygame.joystick import Joystick as pgJoystick
# pgEvent.init()
pygame.init()
pgDisplay.init()
pgJoystick.init()
pgTime.Clock().tick(1)

class Rig(Host):
    def __init__(self, hostname=LOCALHOST, port=SERVER_PORT, **kwargs):
        Host.__init__(self, hostname, port, **kwargs)
        self.gadgets = {}
        self.hostname = hostname
        self.port = port
        self.power = False
        self.is_pygame_rig = False
        self.pygame_gadgets = {}
        
        # trying SIMO event handling
        self.event_handlers = defaultdict(list)
        # self.event_handlers.setdefault([])
        
    
    def add_gadget(self, gadget_name):
        config = loaders.load_gadget(gadget_name)
        gadget_cls = getattr(
            gadget_shelf, config['type'], None)
        assert gadget_cls is not None, f"Gadget type {config['type']} does not exist"
        gadget_cls = gadget_cls(config)
        # logging.debug(f'{gadget_name}.__bases__ = {gadget_cls.__class__}')
        
        # check if gadget class requires pygame
        if 'pygame' in str(gadget_cls.__class__).lower():
            self.is_pygame_rig = True
            self.pygame_gadgets.update({
                gadget_cls.pygame_name:gadget_cls
            })
        
        # TODO - trying to handle multiple gadgets from the same config
        # check that gadget name does not already exist
        # if so, append index to name
        # gadget_name_idx = 1
        # _gadget_name_idx = gadget_name
        
        # while self.gadgets.get(_gadget_name_idx,None) is not None:
            #   _gadget+
            #   gadget_name_idx += 1
        
        
        self.gadgets.update({
            gadget_name:gadget_cls
        })
        return self.gadgets
    
    def _get_gadget_namespace(self, gadget):
        return self.gadgets.get(gadget).namespace
        
    def add_message(self, tx_gadget, rx_gadget, msg_func):
        # logging.debug('add_message',tx_gadget, rx_gadget, msg_func)
        
        tx_namespace, rx_namespace = map(
            self._get_gadget_namespace,
            [tx_gadget, rx_gadget])
        msg_func = getattr(r0b0_msgs,msg_func)
        input_event = msg_func()['event']
        print( tx_namespace, rx_namespace, input_event)
        def func_emit(data):
            # if not isinstance(data,dict): data = pickle.loads(data)
            
            # wrap the data into the gadget's expected message object
            emit_data = self.gadgets[
                rx_gadget].message(
                **msg_func(data))
            output_event = emit_data.event
            # assemble the output to emit
            emit_kwargs = dict(
                event=output_event,
                data={
                    'event':output_event,
                    'msg':pickle.dumps(emit_data)},
                to=None,
                namespace=rx_namespace
            )
            logging.debug(f'func_emit {emit_kwargs}')
            self.emit(**emit_kwargs)
            
        input_handlers = self.event_handlers.get(input_event,[])
        input_handlers.append(func_emit)
        self.event_handlers.update({
            input_event:input_handlers
        })
        # 
        self.on_event(
            input_event,
            # handler=func_emit,
            handler=partial(
                self.multi_handler,
                input_event=input_event),
            namespace=tx_namespace
        )

    def multi_handler(self,data,input_event):
        for handler_func in self.event_handlers.get(input_event,[]):
            handler_func(data)
        
    def pygame_event_handler(self):
        T_LAST_EVENT = 0
        T_COOLDOWN = 300/(10e3)
        while True:
            if (time.time()-T_LAST_EVENT)<T_COOLDOWN: continue
            T_LAST_EVENT = time.time()
            for event in pgEvent.get():
                _event_name = pgEvent.event_name(event.type).lower()
                _event_dict = event.__dict__
                pygame_name = ''
                
                if 'joy' in _event_name:
                    pygame_name = f'joy_{_event_dict.get("joy","")}'
                    
                elif 'key' in _event_name:
                    pygame_name = 'keys'
                    pass
                # logging.debug(pygame_name)
                event_gadget = self.pygame_gadgets.get(
                    pygame_name,None)
                emit_dict =  dict(
                    event=_event_name,
                    data=_event_dict,
                )
                # logging.debug(event_gadget)
                # logging.debug(_event_name)
                # logging.debug(pygame_name)
                if event_gadget is not None:
                    # logging.debug(event_gadget.namespace)
                    emit_dict.update(dict(
                        namespace=event_gadget.namespace
                    ))
                    # logging.debug(emit_dict)
                    event_gadget.emit(**emit_dict)
                break
                    
    def power_on(self,):
        self.start()
        logging.debug(self.gadgets.values())
        [g.start() for g in self.gadgets.values()]
        self.power = True
            
    def power_off(self,*args,**kwargs):
        assert self.power or self.is_alive(), "Rig not powered on"
        self.disconnect()
        [g.disconnect() for g in self.gadgets.values()]
        self.power = False
        
