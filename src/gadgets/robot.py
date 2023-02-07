from src.utils.loaders import load_pickle, dump_pickle

from .gadget import Gadget, Message
from dynamixel_python import \
    DynamixelManager, DynamixelMotor, \
    ReadError
import pickle
from collections import OrderedDict
from socketio import ClientNamespace
import numpy as np

class MotorMessage(Message):
    def __init__(self, event, value, motor_id, **kwargs):
        # breakpoint()
        Message.__init__(self,**kwargs)
        self.event = event
        self.value = value
        self.motor_id = motor_id    
    

class Robot(Gadget, DynamixelManager):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        DynamixelManager.__init__(self,
            # usb_port=self.config['usb_port'],
            usb_port=self.config.get('usb_port', '/dev/tty.usbserial-FT1SF1UM')
            # baud_rate=self.config['baud_rate'],
            baud_rate=self.config.get('baud_rate',57600),
            )
        self.name = config['name']
        # self.motors = OrderedDict()
        self.motors = {}
        self.motors.setdefault(None)
        self.motors_by_id = {}
        self.motors_by_id.setdefault(None)
        if self.config.get('motors', False):
            self.add_motors_from_config(self.config['motors'])
        self.power_up()
        self.kinematic_function = ''
        self.message = MotorMessage
        print(self.__dict__['namespace'])
        self.on('position',
                handler=self.position_event,
                namespace=self.namespace)
        self.on('velocity',
                handler=self.velocity_event,
                namespace=self.namespace)

    # @Gadget.check_msg
    @load_pickle
    def position_event(self,data):
        # msg = pickle.loads(msg)
        # print(msg.motor_id)
        # print(msg=msg)
        msg = data['msg']
        if not isinstance(msg.motor_id,list):
            msg.motor_id = [msg.motor_id]
            msg.value = [msg.value]
        for motor_id, motor_value in zip(msg.motor_id,msg.value):
            motor = self.motors_by_id.get(motor_id,None)
            if motor is None: 
                print(f"No motor ID {motor_id} found, skipping")
                return
            # assert motor is not None, f"Motor {msg.motor_id} does not exist"
            # TODO - set motor control mode
            motor.set_torque_enable(True)
            motor.set_goal_position(int(motor_value))

    # @Gadget.check_msg
    @load_pickle
    def velocity_event(self, msg):
        motor = self.motors_by_id.get(msg.motor_id,None)
        assert motor is not None, f"Motor {msg.motor_id} does not exist"
        self._move_motor_id(msg.motor_id, msg.value)

    def from_config(self, config): 
        for motor, motor_param in config.items():
            self.add_motor(motor, motor_param)
    
    def add_motors_from_config(self, motor_config: list):
        for motor in self.config['motors']:
            dxl_motor = self.add_dynamixel(
                    dxl_name=motor['name'],
                    dxl_id=motor['id'],
                    dxl_model=motor['model'],
            )
            # TODO - set motor control mode
            # https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
                   
            # TODO - use something like a DataFrame instead
            self.motors.update({
                motor['name']:dxl_motor
            })
            # TODO - catch if two motors are on the same id
            self.motors_by_id.update({
                motor['id']:dxl_motor
            })
        
    def power_up(self):
        
        self.init()

    # def id_to_list(self, id, **kwargs):
    def set_compliant(self, compliant=True):
        # for motor in list(self.motors.items()):
        #     motor.set_torque_enable(~compliant)
        if compliant:
            self.enable_all()
        else:
            self.disable_all()

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
        
    def _move_motor_id(self, id, position):
        self.motors_by_id[id].set_torque_enable(True)
        self.motors_by_id[id].set_goal_position(int(position))

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