import unittest
# from .gadget_socket import *
# from .gadget_socket import Socket
# server.start_server('localhost',8080)
import socketio
from src.cables.gadget_socket import Socket, MIDISocket
from .gadget import Gadget, MIDIController, Robot
import mido
import json
from src.utils import loaders
from src.config import CONFIG_DIR
import time
import random
import copy


LOCALHOST = 'localhost'
MIDI_HOST = LOCALHOST
ROBOT_HOST = LOCALHOST
SERVER_PORT = 8080
MIDI_PORT = 9000
ROBOT_PORT = 9090
port_ctr = 9000


sio = socketio.Client()
# sio.connect(f"http://{LOCALHOST}:{SERVER_PORT}")

class GadgetTest(unittest.TestCase):

    def setUp(self) -> None:
        global port_ctr
        port_ctr += 1
        self.port_ctr = port_ctr
        # print(port_ctr)
        self.gadget = Gadget({})
        # print(self.socket.socket_type)
        # assert False
        pass
    
    def test_fail(self):
        # assert False
        pass

    def test_connect(self, hostname=LOCALHOST, port=8080):
        # self.gadget.add_socket(MIDISocket(host='localhost',port=8080))
        self.gadget.connect(
            # hostname=hostname,
            # port=port
            hostname=LOCALHOST,
            port=SERVER_PORT
        )
    
    def tearDown(self):
        # print(self.gadget.type)
        self.gadget.disconnect()
        pass

@unittest.skip('')
class MIDIGadgetTest(GadgetTest):
    def setUp(self):
        super().setUp()
        # config = loaders.load_yaml(str(CONFIG_DIR / 'opz.yaml'))
        config = loaders.load_config('opz')
        self.gadget = MIDIController(
            config=config,
            hostname=MIDI_HOST,
            port=port_ctr)
        # 
        pass
    
    # def test_connect(self):
    #     super().test_connect()
    
    # def test_send(self):
    #     msgs = [
    #         # mido.Message('note_on', note=60, channel=8)
    #     ]
        
    #     # breakpoint()
    #     for i in range(2):
    #         msg = mido.Message(
    #             # f"note_{random.choice(['on','off'])}",
    #             'note_on',
    #             note=random.choice(range(30,90)),
    #             channel=random.choice(range(1,8)))
    #         self.gadget.send(msg)
    #         time.sleep(.5)
            
    def test_midi_event(self):

        print(self.port_ctr)
        self.gadget.connect(LOCALHOST,SERVER_PORT)
        # breakpoint()
        # self.gadget.accept(block=True)
        # print(self.gadget.receive(block=True))
        # self.gadget.accept(block=True)
        pass
    
        
    @unittest.skip('skipping')
    def test_recv(self):
        # return
        # breakpoint()
        # self.gadget.receive
        # self.gadget.accept()
        # breakpoint()
        output = mido.sockets.connect('localhost',8080)
        # breakpoint()
        for i in range(2):
            msg = mido.Message(
                # f"note_{random.choice(['on','off'])}",
                'note_on',
                note=random.choice(range(30,90)),
                channel=random.choice(range(1,8)))
            output.send(msg)
            time.sleep(.5)
            
        # mido.messages.BaseMessage().
    
    def tearDown(self):
        # print(self.gadget.type)
        # self.gadget.close()
        super().tearDown()
        
class RobotGadget(GadgetTest):
    def setUp(self):
        super().setUp()
        config = loaders.load_config('blossom')
        self.gadget = Robot(
            config=config,
            hostname=ROBOT_HOST,
            port=port_ctr,
            )
        pass
    
    def test_connect(self):
        # super().test_connect()
        pass
        
    def test_midi_event(self):
        print(self.gadget)
        
        self.gadget.connect('localhost',8080)
        # self.gadget.emit('midi','testing')
        # self.gadget.wait()
        breakpoint()
        pass
    
    def tearDown(self):
        super().tearDown()
        pass
    