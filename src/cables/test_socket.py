import unittest
# from .gadget_socket import *
from .gadget_socket import Socket
from .midi_socket import MIDISocket
import mido
import json

class SocketTest(unittest.TestCase):

    def setUp(self) -> None:
        self.socket = Socket()
        # print(self.socket.socket_type)
        # assert False
        pass
    
    def test_fail(self):
        # assert False
        pass

    def test_connect_to_server(self):
        pass
    
    def tearDown(self):
        self.socket.close()
        pass


class MIDISocketTest(SocketTest):

    def setUp(self):
        # assert False
        super().setUp()
        self.socket = MIDISocket(
            port_name='OP-Z Bluetooth',
            channel=1
        )

        pass
    
    def test_fail(self):
        # assert False
        pass

    def test_pack_midi_message(self):
        print(mido.get_input_names())
        midi_message = mido.Message('note_on', note=60)
        
        
    
    
    def tearDown(self):
        super().tearDown()
        pass