import socket

class socket(socket.socket):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.socket_type = None # midi, bt, http
        self.gadget = None # midi controller, robot, gamepad, phone

    def bind(self):
        pass 

    # def sendmsg(self):
    #     pass

    def recv(self):
        pass