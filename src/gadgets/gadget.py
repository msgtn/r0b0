class Gadget(object):
    def __init__(self):
        self.type = ''
        pass

    def recv(self):
        pass

    def send(self):
        pass

class MIDIController(Gadget):
    def __init_(self, **kwargs):
        super.__init__(**kwargs)

class Robot(Gadget):
    def __init_(self, **kwargs):
        super.__init__(**kwargs)
        self.name = ''
        self.motors = []
        self.kinematic_function = ''

    def load_config(self):
        pass

    def recv(self):
        pass
        # motor_positions = def handle_kinematics(self):
        pass
    

class Phone(Gadget):
    def __init_(self, **kwargs):
        super.__init__(**kwargs)

class GamePad(Gadget):
    def __init_(self, **kwargs):
        super.__init__(**kwargs)

