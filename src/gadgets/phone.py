from src.config import CSR_PEM, KEY_PEM
from .gadget import Gadget, Message
from collections import OrderedDict
from socketio import ClientNamespace
import numpy as np

class Phone(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.namespace='/'
        # Thread.__init__(self,)
        
    def connect(self):
        pass

    def _connect(self):
        pass