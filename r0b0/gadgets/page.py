from r0b0.config import CSR_PEM, KEY_PEM
from .gadget import Gadget, Message
from r0b0 import logging
from r0b0.utils.loaders import load_msg

from collections import OrderedDict
from socketio import Client, ClientNamespace
import numpy as np
from threading import Thread
from flask import render_template
import time

import pickle

class Page(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.route_urls = config.get('route_urls',{})
        self.event_kwargs = config.get('event_kwargs',{})
        self.on('*',
            handler=self.on_catch_all,
            namespace=self.namespace,
            )
        
    @load_msg
    def on_catch_all(self,data):
        event = data.get('event','unknown event')
        logging.debug(f'Page {self.name} received {event}')
        self.emit(
            event=event,
            data=data,
            namespace=self.namespace,
        )
        
    # def add_emit(self,event,**kwargs):
    #     self.on(
    #         event,
    #         **kwargs,
    #     )
        
    @load_msg
    def on_record(self,data):
        logging.debug(data)

    def start(self):
        Thread.start(self)
        
        # sleep to wait until connection is established
        logging.debug('Connecting Page...')
        time.sleep(2)
        
        # route urls
        for _route,_url in self.route_urls.items():
            self.emit(
                event='add_url',
                data={'route':_route,'url':_url}
            )
            
        # define event handlers
        for _event,_kwargs in self.event_kwargs.items():
            if _kwargs is None: _kwargs={}
            _kwargs.update(dict(
                namespace=self.namespace
            ))
            logging.debug(f'{_event},{_kwargs}')
            self.emit(
                event='add_emit',
                data={
                    'event':_event,
                    'kwargs':_kwargs},
            )
            self.on(
                _event,
                handler=getattr(self,
                    f"on_{_event}",
                    self.on_catch_all),
                    # lambda msg: print(f"Page received {_event} event")),
                namespace=self.namespace,
            )
        
        
class MobilePage(Page):
    def __init__(self, *args, **kwargs):
        Page.__init__(self, *args, **kwargs)

    @load_msg
    def on_device_motion(self,data):
        self.emit(
            event='device_motion',
            data=data,
            namespace=self.namespace,
            )