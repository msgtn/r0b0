from abc import abstractmethod
import pickle

from r0b0.kinematics.blsm import device_motion2dxl_motor, \
    device_motion2dxl_motor320, \
    device_motion2arduino_motor

class Cable(object):
    def __init__(self,):
        self.input_event = ''

    @abstractmethod
    def __call__(self, data: dict):
        """
        When the Cable is called, it will convert an input dictionary into an output dictionary

        :param data: _description_
        :return: _description_
        """
        return {}

class Motion2MotorCable(Cable):
    """
    Converts phone's device motion into motor positions for Blossom
    """
    def __init__(self,):
        # super().__init__()
        self.input_event = 'device_motion'

    def __call__(self, data):
        return {
            'event':'position',
            'value':device_motion2dxl_motor(data),
            'motor_id':[1,2,3,4],
            'absolute':True
        }

class Key2MouseCable(Cable):
    """
    Converts key presses to absolute mouse positions
    """
    def __init__(self,):
        self.input_event = 'keydown'
    
    def __call__(self, data):
        key2pos_dict = {
            'q':[100,100],
            'w':[500,100],
            'e':[900,100],
            'a':[100,400],
            's':[500,400],
            'd':[900,400],
            'z':[100,700],
            'x':[500,700],
            'c':[900,700],
        }
        # key2pos_dict.setdefault([500,400])
        [x,y] = key2pos_dict.get(data['unicode'],[500,400])
        print(x,y)
        return {
            'event':'mouse_place',
            'x':x,
            'y':y,
        }

class MidiRel2PositionCable(Cable):
    """
    Converts relative MIDI_CC messages (increment/decrement) into relative motor positions
    for OpenArm
    """
    def __init__(self,):
        self.input_event = 'midi_cc'

    def __call__(self, data):
        msg = pickle.loads(data['msg'])

        # Map increment (1) / decrement (127)
        value_dict = {
            1:1,
            127:-1
        }
        # Scale the values for each motor
        scale_dict = {
            1:600,
            2:600,
            3:600,
            4:300   # Rotate the wrist less
        }
        value_scale = scale_dict.get(msg.control,300)
        value_dict.setdefault(0)
        
        return {
            'event':'position',
            'value':value_dict.get(msg.value,0)*value_scale,
            'motor_id':msg.control,
            'absolute':False
        }