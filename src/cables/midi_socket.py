from .gadget_socket import Socket
# from .socket import 
from socketio import Client
import mido
import json




class MIDISocket(Socket):
    def __init__(self, channel: int=-1, **kwargs):
        super().__init__(socket_type='midi', **kwargs)
        # self.port_name = port_name
        self.channel = channel
        self.port = None

    def bind_midi(self,port_name):
        self.port = mido.open_input(self.port_name)
        pass

    def bind_socket(self,socket):
        # connect to server
        pass

    #@ sio stuff
    def send(self):
        pass
    def pack(self, message):
        pass


if __name__=="__main__":
    pass