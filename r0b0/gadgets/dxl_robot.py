from threading import Thread
import time
import numpy as np
from functools import partial
from r0b0.utils.loaders import load_msg, dump_msg

from .gadget import Gadget, Message, logging
from dynamixel_python import \
    DynamixelManager, DynamixelMotor, \
    ReadError

BAUD_DICT = {
    57600:1,
    115200:2,
    1000000:3,
    9600:0
}
# for xl330-m{288,077}
# might be different for others
MODE_DICT = {
    'position':3,
    'velocity':1,
    'extended_position':4,
}

T_LAST_CMD = 0
T_COOLDOWN = 300/10e3

class MotorMessage(Message):
    def __init__(self, **kwargs):
        Message.__init__(self,**kwargs)
        # self.event = event
        # self.value = value
        # self.motor_id = motor_id    

class DynamixelRobot(Gadget, DynamixelManager):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self,
            config,
            request_timeout=0.1,
            **kwargs)
        DynamixelManager.__init__(self,
            usb_port=self.config.get(
                'usb_port', '/dev/tty.usbserial-FT1SF1UM'),
            baud_rate=self.config.get(
                'baud_rate',57600),
            )
        self.name = config['name']
        if self.config.get('motors', False):
            self.motors_by_id = self.add_motors_from_config(
                self.config['motors'])
        # TODO - overwrite dxl_dict with this?
        

        self.power_up = self.init
        self.power_up()
        self.enable,self.disable = self.enable_all,self.disable_all
        # self.enable = partial(self.set_param,)
        self.enable()

        self.message = MotorMessage
        
        handled_events = ['param','position',
                          'velocity'
                          ]
        for _event in handled_events:
            self.on(_event,
                handler=getattr(self,f'{_event}_event'),
                namespace=self.namespace
            )
        
        self.motion_thread = Thread(
            target=self._motion_thread
        )
        logging.debug(f'namespace {self.namespace}')
        
        self.set_param = partial(self.access_param,
            rw_mode='set')
        self.get_param = partial(self.access_param,
            rw_mode='get')
        
    def _motion_thread(self):
        # continuously check the 
        # "is moving" flag and emit events
        pass
    
    def access_param(self,param,motor_id_kwargs,rw_mode='set'):
        for motor_id,motor_kwargs in motor_id_kwargs.items():
            # _motor = self.dxl_dict.get(str(motor_id),None)
            _motor = self.motors_by_id.get(motor_id,None)
            
            # TODO - packet should just be in a dict
            # with 'data' as the default arg
            # ps2014.set_param('goal_velocity',{1:{'data':-1600}})
            if _motor is not None:
                getattr(
                    _motor,f'{rw_mode}_{param}'
                    )(**motor_kwargs)
        
    # def turn_to_list(self):
    def _var2list(self,*_vars):
        _return_list = []
        for _var in _vars:
            if not isinstance(_var,list):
                _return_list.append([_var])
            else:
                _return_list.append(_var)
        return _return_list
    
    def _msg2kwargs(self,msg):
        if getattr(msg, 'motor_kwargs', None) is None:
            msg.__dict__.update({
                'motor_kwargs':msg.value
            })
        [motor_ids,motor_kwargs] = self._var2list(
            msg.motor_id,msg.motor_kwargs)
        motor_kwargs = [{'data':mk} for mk in motor_kwargs]
        return {
            m_id:m_kwargs for m_id,m_kwargs in zip(
                motor_ids,motor_kwargs)
        }
        
    @load_msg
    def _position_event(self,data):
        # msg is a data object
        msg = data['msg']
        motor_id_kwargs = self._msg2kwargs(msg)
        
        if not getattr(msg,'absolute')=='absolute':
            # handle calculation of relative position
            # get current position
            # calculate differences
            # update values
            pass
        self.enable()
        self.set_param(
            'goal_position',
            motor_id_kwargs,
        )
        
    @load_msg
    def velocity_event(self,data):
        # msg is a data object
        msg = data['msg']
        motor_id_kwargs = self._msg2kwargs(msg)
        
        if not getattr(msg,'absolute')=='absolute':
            # handle calculation of relative velocity
            velocity_present = self.get_param(
                'present_velocity',
                {m_id:{} for m_id in motor_id_kwargs.keys()}
            )
            
            # get current position
            # calculate differences
            # update values
            pass
        self.enable()
        self.set_param(
            'goal_velocity',
            motor_id_kwargs,
            # {m_id:{'data':m_k} for m_id,m_k in motor_id_kwargs},
        )
    
    @load_msg
    def param_event(self,data):
        msg = data['msg']
        if not isinstance(msg.motor_id,list):
            msg.motor_id = [msg.motor_id]
            msg.value = [msg.value]

    @load_msg
    def position_event(self,data):
        msg = data['msg']
        if not isinstance(msg.motor_id,list):
            msg.motor_id = [msg.motor_id]
            msg.value = [msg.value]
        for motor_id, motor_value in zip(msg.motor_id,msg.value):
            motor = self.motors_by_id.get(motor_id,None)
            if motor is None:
                logging.debug(f"No motor ID {motor_id} found, skipping")
                continue
            motor.t_last_cmd = time.time()
            
            # assert motor is not None, f"Motor {msg.motor_id} does not exist"
            # TODO - set motor control mode
            # motor.set_torque_enable(True)
            motor.set_profile_velocity(16000)
            motor.set_goal_position(int(motor_value))
            motor.believed_position = int(motor_value)
    
    def add_motors_from_config(self, motor_config: list):
        self.motors_by_id  = {}
        self.motors_by_id.setdefault(None)
        for motor in self.config['motors']:
            dxl_motor = self.add_dynamixel(
                dxl_name=motor['name'],
                dxl_id=motor['id'],
                dxl_model=motor['model'],
            )
            
            dxl_motor = Motor.from_motor(dxl_motor)
            
            # cannot set operating mode here - must first init
            # if motor.get('mode',False):
            #     motor_mode = MODE_DICT.get(motor['mode'],None)
            #     if motor_mode is None:
            #         logging.warn(
            #             f'No mode {motor["mode"]} \
            #             for motor {motor["name"]}, \
            #             setting to "position."'
            #         )
            #     breakpoint()
            #     dxl_motor.set_operating_mode(motor_mode)
            
            self.motors_by_id.update({
                int(motor['id']):dxl_motor
            })
            
        return self.motors_by_id

    def _deg2dxl(self, deg, deg_range=[-150,150], dxl_range=[0,4096]):
        return int(np.interp(deg, deg_range, dxl_range))

class Motor(DynamixelMotor):
    def __init__(self,**kwargs):
        # super().__init__(self,**kwargs)
        self.__dict__.update(**kwargs)
        self.mode = kwargs.get('mode','position')
        self.t_last_cmd = 0
        
        self.enable = partial(self.set_param,
            param='torque_enable',
            value=True)
        self.disable = partial(self.set_param,
            param='torque_enable',
            value=False)
        
    @classmethod
    def from_motor(cls, dxl_motor: DynamixelMotor, **kwargs):
        # self.__dict__.update(dxl_motor.__dict__)
        
        return cls(**dxl_motor.__dict__)
        # self.t_last_cmd = time.time()
        # self.believed_position = 0
        
        # return self
     
    def set_param(self,param,value):
        getattr(self, f"set_{param}")(value)

    def set_mode(self, mode):
        self.disable()
        self.set_param('operating_mode',mode)
        self.enable()