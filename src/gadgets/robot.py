from .gadget import Gadget, Message
from dynamixel_python import \
    DynamixelManager, DynamixelMotor, \
    ReadError
import pickle
from collections import OrderedDict
from socketio import ClientNamespace
import numpy as np


class Robot(Gadget, DynamixelManager):
    def __init__(self, **kwargs):
        Gadget.__init__(self, **kwargs)
        DynamixelManager.__init__(self,
            usb_port=self.config['usb_port'],
            baud_rate=self.config['baud_rate'])
        self.name = ''
        self.motors = OrderedDict()
        self.add_motors_from_config(self.config['motors'])
        self.power_up()
        # self.set_compliant(False)
        self.kinematic_function = ''
        self.message = MotorMessage
        self.on('*',self.any_event)
        self.on('motor',self.motor_event)
        
        
    def connect(self, *args, **kwargs):
        Gadget.connect(namespace='/robot',*args,**kwargs)
        
    def any_event(self):
        print('event')
        
    def connect_event(self,):
        print(f"Connected to ")
        
    def motor_event(self, message):
        motor_message = pickle.loads(message)
        pass
        
    def add_dynamixel(self, motor_name: str, motor_id: int, motor_model: str, **kwargs):
    
        motor = DynamixelManager.add_dynamixel(
            motor_name,
            motor_id,
            motor_model,
            **kwargs)
        if not motor.ping():
            raise BaseException(f"Motor {motor_name} not configured correctly")
        motor.set_operating_mode(3)
        motor.set_profile_velocity(262)
        motor.set_torque_enable(True)
        return motor
        
    def from_config(self, config): 
        for motor, motor_param in config.items():
            self.add_motor(motor, motor_param)
    
    def add_motors_from_config(self, motor_config: list):
        for motor in self.config['motors']:
            self.motors.update({
                motor['name']:self.add_dynamixel(
                    motor_name=motor['name'],
                    motor_id=motor['id'],
                    motor_model=motor['model'],
                    
            )})
        
    def power_up(self):
        self.init()

    # def id_to_list(self, id, **kwargs):
    def set_compliant(self, compliant=True):
        for motor in list(self.motors.items()):
            motor.set_torque_enable(~compliant)

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

    def load_sequence(self, seq_fn, rad=True, force=True):
        pass
    def add_sequence(self, seq):
        pass


class Motor(DynamixelMotor):
    def __init__(**kwargs):
        super().__init__(**kwargs)
        
        
class MotorMessage(Message):
    def __init__(self, msg_type, value, motor_id, **kwargs):
        Message.__init__(**kwargs)
        self.msg_type = msg_type
        self.value = value
        self.motor_id = motor_id    
    
def midicc2motor(tx_msg, ):
    return MotorMessage(
        type='position',
        value=tx_msg.value*4096//127,
        motor_id=tx_msg.channel+1,
    )
    
def midinote2motor(tx_msg, ):
    return MotorMessage(
        type='position',
        value=np.interp(tx_msg.value, [30,60], [0,4096]),
        motor_id=tx_msg.channel+1,   
    )