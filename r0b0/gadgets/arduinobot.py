from .gadget import Gadget, Message
from r0b0 import logging
from r0b0.utils import loaders
from r0b0.utils.loaders import load_pickle, dump_pickle

import time
# from dynamixel_python import \
#     DynamixelManager, DynamixelMotor, \
#     ReadError
import pickle
from collections import OrderedDict
from socketio import ClientNamespace
import numpy as np
# from Arduino import Arduino
from pyfirmata import Arduino, util

# Needs to upload the StandardFirmata to the board first

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
            self.config['usb_port'],
            # baud=self.config['baud_rate'],
            # timeout=self.config['timeout'],
            )
        self.motors_by_id = {}
        
        if self.config.get('motors', False):
            self.add_motors_from_config(self.config['motors'])
        self.on('position',
                handler=self.position_event,
                namespace=self.namespace)
        self.on('velocity',
                handler=self.velocity_event,
                namespace=self.namespace)
        return
        self.name = config['name']
        self.motors = {}
        self.motors.setdefault(None)
        self.motors_by_id = {}
        self.motors_by_id.setdefault(None)
        self.power_up()
        self.kinematic_function = ''
        self.message = MotorMessage

    @load_pickle
    def position_event(self,data):
        # msg = pickle.loads(msg)
        # print(msg.motor_id)
        # print(msg=msg)
        msg = data['msg']
        # logging.debug(msg)
        if not isinstance(msg.motor_id,list):
            msg.motor_id = [msg.motor_id]
            msg.value = [msg.value]
        for motor_id, motor_value in zip(msg.motor_id,msg.value):
            # motor_value = np.interp(motor_value, [0,4096], [10,160])
            motor = self.motors_by_id.get(motor_id,None)
            if motor is None:
                # print(f"No motor ID {motor_id} found, skipping")
                logging.debug(f'Motor {motor_id} not found, skipping')
                return
            # if (time.time()-motor.t_last_cmd) < T_COOLDOWN:
            #     continue
            # print(motor_value)
            logging.debug('motor_value')
            logging.debug(motor_value)
            motor.write(motor_value)

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
            motor = self.get_pin(f'd:{motor_id}:s')
            
            # self.Servos.attach(motor_id)
            # self.motors.update({
            #     motor['name']:motor_id
            # })
            
            # for i in range(0,150,15):
            #     motor.write(i)
            #     logging.debug(i)
            #     time.sleep(1)
                
            self.motors_by_id.update({
                motor_id:motor
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


# class Motor(DynamixelMotor):
#     def __init__(**kwargs):
#         super().__init__(**kwargs)
        
        
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

if __name__=="__main__":
    board = ArduinoBot(
        config=loaders.load_gadget('ardubot')
    )
    # board = Arduino('9600',port='/dev/cutimeout=10)
    LED_PIN = 13
    board.pinMode(LED_PIN,'OUTPUT')
    import time
    while True:
        board.digitalWrite(LED_PIN, "LOW")
        time.sleep(1)
        board.digitalWrite(LED_PIN, "HIGH")
        time.sleep(1)
    breakpoint()