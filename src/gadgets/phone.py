from src.config import CSR_PEM, KEY_PEM
from .gadget import Gadget, Message

from collections import OrderedDict
from socketio import Client, ClientNamespace
import numpy as np
from threading import Thread
from flask import render_template
import time

import pickle
import logging
logging.basicConfig(
    filename='phone.log',
    encoding='utf-8',
    level=logging.DEBUG,
)

logging.debug('warning'.upper())

class Phone(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.route_urls = config.get('route_urls',{})
        self.event_kwargs = config.get('event_kwargs',{})
        self.on('*',
            handler=lambda msg: self.emit(**msg))
        
    def add_emit(self,event,**kwargs):
        self.on(
            event,
            **kwargs,
        )
        
    # @dump_pickle
    def on_record(self,msg):
        # self.emit()
        # print(msg)
        logging.debug(msg)
        
    # @dump_pickle
    def on_device_motion(self,msg):
        # print(msg)
        self.emit(
            event='device_motion',
            data=msg,
            namespace=self.namespace,
            )
        
        
    def start(self):
        Thread.start(self)
        
        # sleep to wait until connection is established
        print('Connecting Phone...')
        time.sleep(2)
        
        # route urls
        for _route,_url in self.route_urls.items():
            self.emit(
                event='add_url',
                data={'route':_route,'url':_url}
            )
            
        # define events
        for _event,_kwargs in self.event_kwargs.items():
            self.emit(
                event='add_emit',
                data={'event':_event,'kwargs':_kwargs},
                # namespace=self.namespace,
            )
            self.on(
                _event,
                handler=getattr(self,
                    f"on_{_event}",
                    lambda msg: print(f"Phone received {_event} event")),
                namespace=self.namespace,
            )
        