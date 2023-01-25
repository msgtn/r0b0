from src.config import CSR_PEM, KEY_PEM
from .gadget import Gadget, Message
from collections import OrderedDict
from socketio import ClientNamespace
import numpy as np
from threading import Thread
from flask import render_template
import time
from socketio import Client

class Phone(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        # self.namespace='/'
        # Thread.__init__(self,)
        
    # def connect(self):
    #     pass

    # def _connect(self):
    #     pass
    
    def start(self):
        # breakpoint()
        Thread.start(self)
        
        time.sleep(1)
        
        route_urls = {
            '/':'index',
            '/broadcast':'broadcast',
        }
        for _route,_url in route_urls.items():
            self.emit(
                'add_url',
                {'route':_route,'url':f"{_url}.html"}
            )
        