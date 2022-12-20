import numpy as np
# import pypot.robot
# from . import sequence
from socketio import \
    Client, AsyncClient, \
    Server, AsyncServer
import collections
import time
from dynamixel_python import DynamixelManager, ReadError

motor_lut = {
    'XL-320':'xl320',
    'XL-330':'xl330-m288'
}

# sio = AsyncClient()
# server = Server()
client = Client()


class Robot(DynamixelManager):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.motors = { }
        self.motor_channels = {} 
        self.seq_list = collections.OrderedDict()
        self.compliant = True
        self.power_up()
        self.set_compliant(False)
        self.reset_pos = {
            'tower_1':0,
            'tower_2':0,
            'tower_3':0,
            'base':0,
            'ears':100,
            'left_arm':100,
            'right_arm':-100,
            'tail':0
        }
        self.range_pos = {
            'tower_1':(-40,140),
            'tower_2':(-40,140),
            'tower_3':(-40,140),
            'base':(-150,150),
            'ears':(0,140),
            'left_arm':(-150,150),
            'right_arm':(-150,150),
            'tail':(-150,150)
        }
        self.believed_motor_pos = self.reset_pos
        self.reset_position()
        # self.sio = AsyncClient()

    @client.event(namespace='/midi')
    def midi_event(sid, data):
        print(data)
        return "OK", 123

    @client.event(namespace='/http')
    def http_event(sid, data):
        return "OK", 123

    @client.event(namespace='/gamepad')
    def gamepad_event(sid, data):
        return "OK", 123

    def from_config(self, config): 
        for motor, motor_param in config.items():
            self.add_motor(motor, motor_param)

    def add_motor(self, motor,motor_param):
        motor_obj = self.add_dynamixel(motor, **motor_param)
        # motor_obj.set_operating_mode(3)
        self.motors.update({motor:motor_obj} )
        self.motor_channels.update({motor_param['dxl_id']:motor_obj})

    def power_up(self):
        self.init()


    # def id_to_list(self, id, **kwargs):
    def set_compliant(self, compliant=True):
        for motor in list(self.motors.items()):
            motor.set_torque_enable(compliant)

    def move_motor_name(self, name, position):
        if isinstance(name, list):
            for _name,_position in zip(name,position):
                self._move_motor_name(_name, _position)
        else:
            self._move_motor_name(name, position)
    

    def move_motor_id(self, id, position):
        if isinstance(id, list):
            for _id,_position in zip(id,position):
                self._move_motor_id(_id, _position)
        else:
            self._move_motor_id(id, position)
    
    def _move_motor_id(self, id, position):
        self.motor_channels[id].set_torque_enable(True)
        self.motor_channels[id].set_goal_position(position)

    def _move_motor_name(self, name, position):
        self.motors[name].set_torque_enable(True)
        self.motors[name].set_goal_position(position)

    def motor_fn(self, id, fn, **kwargs):
        try:
            return getattr(self.motor_channels[id], fn)(**kwargs)
        except ReadError:
            print(f'Motor {id} blocked')

    def _deg2dxl(self, deg, deg_range=[-150,150], dxl_range=[0,4090]):
        return int(np.interp(deg, deg_range, dxl_range))

    def goto_position(self,motor_pos, delay=0.1, wait=False):
        available_motors = list(self.motors.keys())
        for motor_name, position in motor_pos.items():
            if motor_name not in available_motors:
                continue
            self.move_motor_name(motor_name,self._deg2dxl(position))
            # self.move_motor

    def get_motor_pos(self):
        
        return {motor_name:motor.get_present_position() \
            for motor_name, motor in self.motors.items()}

    def reset_position(self):
        self.goto_position(self.reset_pos)

    def reconfig(self, config):
        pass

    # def set_compliant(self, compliant=True):
    def load_sequence(self, seq_fn, rad=True, force=True):
        pass
    def add_sequence(self, seq):
        pass
