import mido
from mido.sockets import PortServer, connect
from src.cables.gadget_socket import Socket, MIDISocket
from socketio import Client
sio = Client()


# class Gadget(object):
class Gadget(Client):
    def __init__(self, hostname='localhost', port=8000, **kwargs):
        super().__init__()
        self.hostname=hostname
        self.port = port
        self.__dict__.update(**kwargs)
        pass

    def connect(self, hostname, port, header='http'):
        # breakpoint()
        print(f"Connecting to {header}://{hostname}:{port}")
        super().connect(f"{header}://{hostname}:{port}")

    def recv(self):
        pass

    def send(self, sockets=None):
        if not sockets:
            for socket in self.sockets:
                socket.send('s')
        pass
    
    def disconnect(self) -> None:
        super().disconnect()
        pass

# class MIDIController(PortServer, Gadget):
# class MIDIController(Gadget, PortServer):
class MIDIController(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, **kwargs)
        # PortServer.__init__(self,
        #     # port=self.config['name'],
        #     host=kwargs['hostname'],
        #     portno=kwargs['port']
        # )
        print(mido.get_input_names())
        self.midi_port = mido.open_ioport(config['port_name'])
        
        
        self.on('midi',self.midi_event)
        
    def add_socket(self, socket: MIDISocket):
        self.sockets.append(socket)
        
        
    def midi_event(self, sid, data):
        print(data)
        pass
    
    def disconnect(self):
        super().disconnect()
        self.midi_port.close()
      
from dynamixel_python import DynamixelManager, ReadError
class Robot(Gadget, DynamixelManager):
    def __init__(self, **kwargs):
        Gadget.__init__(self, **kwargs)
        self.name = ''
        self.motors = []
        self.kinematic_function = ''
        
    def add_motor(self):
        pass

    def load_config(self):
        # load this from yaml or a dictionary
        pass

    def recv(self):
        pass
        # motor_positions = def handle_kinematics(self):
        pass
    
    def close(self):
        # self.inport.close()
        # self.outport.close()
        pass
    
    
    @sio.event(namespace='/midi')
    def midi_event(sid, data):
        print(data)
        return "OK", 123

    @sio.event(namespace='/http')
    def http_event(sid, data):
        return "OK", 123

    @sio.event(namespace='/gamepad')
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


class Phone(Gadget):
    def __init_(self, **kwargs):
        super.__init__(**kwargs)

class GamePad(Gadget):
    def __init_(self, **kwargs):
        super.__init__(**kwargs)

