import unittest
# from .gadget_socket import *
# from .gadget_socket import Socket
# server.start_server('localhost',8080)
import socketio
from src.cables.gadget_socket import Socket, MIDISocket
# from .gadget import Gadget, MIDIController, Robot
from . import Gadget, MIDIController, Robot
import mido
import json
from src.utils import loaders
import time
import random
import copy
import asyncio
from src.config import \
    CONFIG_DIR, \
    LOCALHOST, MIDI_HOST, ROBOT_HOST, \
    SERVER_PORT, MIDI_PORT, ROBOT_PORT

port_ctr = 9000

sio = socketio.Client()

class GadgetTest(unittest.TestCase):

    def setUp(self) -> None:
        global port_ctr
        port_ctr += 1
        self.port_ctr = port_ctr
        self.gadget = Gadget(config={})
        
    def test_fail(self):
        # assert False
        pass

    # @unittest.skip('')
    def test_connect(self, hostname=LOCALHOST, port=8080):
        print(self.gadget)
        self.gadget.connect(
            hostname=LOCALHOST,
            port=SERVER_PORT
        )
    
    def tearDown(self):
        self.gadget.disconnect()
        
        
class MIDIGadgetTest(GadgetTest):
    def setUp(self):
        super().setUp()
        config = loaders.load_config('opz')
        self.gadget = MIDIController(
            config=config,
            hostname=MIDI_HOST,
            port=port_ctr)
        
    def test_connect(self):
        super().test_connect()
        breakpoint()
            
    @unittest.skip('skipping')
    def test_recv(self):
        pass
    
    def tearDown(self):
        super().tearDown()
        
@unittest.skip('')
class RobotGadgetTest(GadgetTest):
    def setUp(self):
        super().setUp()
        config = loaders.load_config('blossom')
        self.gadget = Robot(
            config=config,
            )
        
        
    def test_midi_event(self):
        super().connect()
        self.gadget.wait()
    
    def tearDown(self):
        super().tearDown()
        