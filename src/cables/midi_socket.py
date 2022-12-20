from .socket import Server
import mido

class MIDIServer(Server):
    def __init__(self, channel, **kwargs):
        super.__init__(**kwargs)
        self.channel = channel

    def bind_midi(self,portname):
        self.port = mido.open_input(portname)
        pass

    def bind_socket(self,socket):
        # connect to robot's tcp socket