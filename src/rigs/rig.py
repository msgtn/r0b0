# from src import gadgets
# import src.gadgets
from src import gadgets as gadget_shelf
from src.config import LOCALHOST, SERVER_PORT
from src.utils import loaders
# from src.rigs import logging
# from src.gadgets.rig import start_server
from src.rigs.server import Host
from src.messages import msg_funcs
from multiprocessing import Process
import pickle
from  src import logging
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
    
    def add_gadget(self, gadget_name):
        config = loaders.load_gadget(gadget_name)
        gadget = getattr(gadget_shelf, config['type'], None)
        assert gadget is not None, f"Gadget type {config['type']} does not exist"
        gadget = gadget(config)
        logging.debug(f'{gadget_name}.__bases__ = {gadget.__class__}')
        if 'pygame' in str(gadget.__class__).lower():
            self.is_pygame_rig = True
            self.pygame_gadgets.update({
                gadget.pygame_name:gadget
            })
        
        # check that gadget name does not already exist
        # if so, append index to name
        # gadget_name_idx = 1
        # _gadget_name_idx = gadget_name
        
        # while self.gadgets.get(_gadget_name_idx,None) is not None:
            #   _gadget+
            #   gadget_name_idx += 1
        
        
        self.gadgets.update({
            gadget_name:gadget
        })
        return self.gadgets
    
    def _get_gadget_namespace(self, gadget):
        return self.gadgets.get(gadget).namespace
        
    def add_message(self, tx_gadget, rx_gadget, msg_func):
        # logging.debug('add_message',tx_gadget, rx_gadget, msg_func)
        
        tx_namespace, rx_namespace = map(
            self._get_gadget_namespace,
            [tx_gadget, rx_gadget])
        print( tx_namespace, rx_namespace)
        msg_func = getattr(msg_funcs,msg_func)
        def func_emit(data):
            # if not isinstance(data,dict): data = pickle.loads(data)
            emit_data = self.gadgets[rx_gadget].message(
                **msg_func(data))
            # print(data)
            logging.debug(f"func_emit {data}")
            self.emit(
                event=emit_data.event,
                data={
                    'event':emit_data.event,
                    'msg':pickle.dumps(emit_data)},
                to=None,
                namespace=rx_namespace
            )
        # print('tx',tx_namespace)
        self.on_event(
            msg_func()['event'],
            handler=func_emit,
            namespace=tx_namespace
        )
        
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
                    # print(_event_dict)
                    # pass
                    pygame_name = f'joy_{_event_dict.get("joy","")}'
                    
                elif 'key' in _event_name:
                    pygame_name = 'keys'
                    # pygame_name = ''
                    pass
                # logging.debug(pygame_name)
                event_gadget = self.pygame_gadgets.get(
                    pygame_name,None)
                emit_dict =  dict(
                    event=_event_name,
                    data=_event_dict,
                )
                if event_gadget is not None:
                    emit_dict.update(dict(
                        namespace=event_gadget.namespace
                    ))
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
        
