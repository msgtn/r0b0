# from src.gadgets.gadget import Gadget, MIDIController

from socketio import Client, Server
import json

class Socket(Client):
    def __init__(self, host: str, port: int, socket_type: str = None, **kwargs):
        super().__init__(**kwargs)
        self.socket_type = socket_type # midi, bt, http
        # self.gadgets = []

    def bind(self):
        pass 

    # def add_gadget(self, gadget: Gadget):
    #     self.gadgets.append(gadget)
    # def sendmsg(self):
    #     pass
    
    def send(self,msg):
        pass

    def recv(self):
        pass
        

import mido

class MIDISocket(Socket):
    def __init__(self, channel: int=-1, **kwargs):
        super().__init__(socket_type='midi', **kwargs)

        self.channel = channel
        self.port = None

    def bind_midi(self,port_name):
        # self.port = mido.open_input(self.port_name)
        pass

    def bind_socket(self,socket):
        # connect to server
        pass

    #@ sio stuff
    def send(self):
        pass
    def pack(self, message):
        pass

