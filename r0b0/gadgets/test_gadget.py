import unittest
# from .gadget_socket import *
# from .gadget_socket import Socket
# server.start_server('localhost',8080)
import socketio
# from r0b0.gadget_socket import Socket, MIDISocket
from r0b0.gadgets.phone import Phone
from r0b0.rigs.server import Host
# from .gadget import Gadget, MIDIController, Robot
from . import Gadget, MIDIController, Robot
import mido
import json
from r0b0.utils import loaders
import time
import random
import copy
import asyncio
from r0b0.config import \
    CONFIG_DIR, \
    LOCALHOST, MIDI_HOST, ROBOT_HOST, \
    SERVER_PORT, MIDI_PORT, ROBOT_PORT

port_ctr = 9000

sio = socketio.Client()
host = Host(
    hostname=LOCALHOST,
    port=SERVER_PORT
)
host.start()

# @unittest.skip('')    
class GadgetTest(unittest.TestCase):

    def setUp(self) -> None:
        # global port_ctr
        # port_ctr += 2
        # self.port_ctr = port_ctr
        self.gadget = Gadget(
            config=loaders.load_gadget('testgadget'),
            hostname=LOCALHOST,
            port=SERVER_PORT,
            )
        # self.host = Host(port=self.port_ctr)
        # self.host.start()
        # print(self.gadget.__dict__)
        
    def test_fail(self):
        # assert False
        pass
    
    def test_start(self):
        self.gadget.start()
        # time.sleep(1)
        retry = 0
        # while not self.gadget.connected and retry<1000:
            # retry += 1
        # self.assertTrue(self.gadget.connected)
        self.assertTrue(self.gadget.is_alive())
        time.sleep(5)
        self.assertTrue(self.gadget.connected)
        # self.assertFalse(self.gadget.connected)
        
    def test_disconnect(self):
        self.test_start()
        self.gadget.disconnect()
        time.sleep(5)
        self.assertFalse(self.gadget.connected)
        
    @unittest.expectedFailure
    def test_disconnect_when_not_connected(self):
        self.gadget.disconnect()
        
    def tearDown(self):
        # print(self.gadget.__dict__)
        if self.gadget.connected: self.gadget.disconnect()
        if self.gadget.is_alive(): self.gadget.join()
        # if host.is_alive(): self.host.join()
        # if host.connected: self.host.disconnect()
        
@unittest.skip('')    
class MIDIGadgetTest(GadgetTest):
    def setUp(self):
        super().setUp()
        config = loaders.load_gadget('opz')
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