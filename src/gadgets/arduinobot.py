from .gadget import Gadget, Message, logging
from dynamixel_python import \
    DynamixelManager, DynamixelMotor, \
    ReadError
import pickle
from collections import OrderedDict
from socketio import ClientNamespace
import numpy as np
from Arduino import Arduino

class MotorMessage(Message):
    def __init__(self, event, value, motor_id, **kwargs):
        # breakpoint()
        Message.__init__(self,**kwargs)
        self.event = event
        self.value = value
        self.motor_id = motor_id    
    

class ArduinoBot(Gadget, Arduino):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        Arduino.__init__(self,
            port=self.config['usb_port'],
            baud=self.config['baud_rate'],
            timeout=self.config['timeout'])
        self.name = config['name']
        self.motors = {}
        self.motors.setdefault(None)
        self.motors_by_id = {}
        self.motors_by_id.setdefault(None)
        if self.config.get('motors', False):
            self.add_motors_from_config(self.config['motors'])
        self.power_up()
        self.kinematic_function = ''
        self.message = MotorMessage
        self.on('position',
                handler=self.position_event,
                namespace=self.namespace)
        self.on('velocity',
                handler=self.velocity_event,
                namespace=self.namespace)

    # @Gadget.check_msg
    def position_event(self,msg):
        msg = pickle.loads(msg)
        # print(msg.motor_id)
        if not isinstance(msg.motor_id,list):
            msg.motor_id = [msg.motor_id]
            msg.value = [msg.value]
        print(msg)
        for motor_id, motor_value in zip(msg.motor_id,msg.value):
            # self.Servos.write(motor_id,motor_value)
            self._move_motor_id(motor_id,motor_value)

    # @Gadget.check_msg
    def velocity_event(self, msg):
        motor = self.motors_by_id.get(msg.motor_id,None)
        assert motor is not None, f"Motor {msg.motor_id} does not exist"
        self._move_motor_id(msg.motor_id, msg.value)

    def from_config(self, config): 
        for motor, motor_param in config.items():
            self.add_motor(motor, motor_param)
    
    def add_motors_from_config(self, motor_config: list) -> None:
        for motor in self.config['motors']:
            motor_id = motor.get('pin',motor['id'])
            logging.debug(f'attaching motor {motor_id}')
            self.Servos.attach(motor_id)
            self.motors.update({
                motor['name']:motor_id
            })
            self.motors_by_id.update({
                motor_id:motor['name']
            })
        
    def power_up(self):
        pass
        # self.init()

    def move_motor_name(self, name, position):
        if isinstance(name, list):
            for _name,_position in zip(name,position):
                self._move_motor_name(_name, _position)
        else:
            self._move_motor_name(name, position)

    def move_motor_id(self, motor_id, position):
        if not isinstance(motor_id, list): 
            motor_id = [motor_id]
            position = [position]
        for _id,_position in zip(motor_id,position):
            self._move_motor_id(_id, _position)
        
    def _move_motor_id(self, motor_id, motor_value):
        self.Servos.write(motor_id, motor_value)

    def _move_motor_name(self, motor_name, motor_value):
        motor_id = self.motors.get(motor_name, None)
        if motor_id is not None:
            self.Servos.write(motor_id,motor_value)

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
        
        
# def midicc2motor(tx_msg, ):
#     return MotorMessage(
#         type='position',
#         value=tx_msg.value*4096//127,
#         motor_id=tx_msg.channel+1,
#     )
    
# def midinote2motor(tx_msg, ):
#     return MotorMessage(
#         type='position',
#         value=np.interp(tx_msg.value, [30,60], [0,4096]),
#         motor_id=tx_msg.channel+1,   
#     )