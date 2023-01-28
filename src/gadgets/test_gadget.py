import unittest
# from .gadget_socket import *
# from .gadget_socket import Socket
# server.start_server('localhost',8080)
import socketio
# from src.gadget_socket import Socket, MIDISocket
from src.gadgets.phone import Phone
from src.rigs.server import Host
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

# @unittest.skip('')    
class GadgetTest(unittest.TestCase):

    def setUp(self) -> None:
        global port_ctr
        port_ctr += 2
        self.port_ctr = port_ctr
        self.gadget = Gadget(config={},port=port_ctr)
        self.host = Host(port=port_ctr)
        self.host.start()
        # print(self.gadget.__dict__)
        
    def test_fail(self):
        # assert False
        pass
    
    def test_start(self):
        # self.gadget.power_on()
        self.gadget.start()
        time.sleep(2)
        self.assertTrue(self.gadget.connected)
        
    def test_disconnect(self):
        self.gadget.test_start()
        assert not self.gadget.connected
        
    @unittest.expectedFailure
    def test_disconnect_when_not_connected(self):
        self.gadget.disconnect()
        
    def tearDown(self):
        self.gadget.join()
        self.host.join()
        
@unittest.skip('')    
class MIDIGadgetTest(GadgetTest):
    def setUp(self):
        super().setUp()
        config = loaders.load_config('opz')
        self.gadget = MIDIController(
            config=config,
            hostname=MIDI_HOST,
            port=port_ctr)
        
    @unittest.skip('skipping')
    def test_recv(self):
        pass
    
    def tearDown(self):
        super().tearDown()

@unittest.skip('')            
class PhoneTest(GadgetTest):
    def setUp(self):
        super().setUp()
        config = loaders.load_gadget('test_phone')
        self.gadget = Phone(config)
        
    # def test_on_device_motion(self):
    def test_on_record(self):
        self.gadget.on_record('test')
        
    @unittest.expectedFailure
    def test_on_record_no_msg(self):
        self.gadget.on_record()