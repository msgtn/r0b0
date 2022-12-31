'''
GVR's design goals for Python
An easy and intuitive language just as powerful as major competitors
Open source, so anyone can contribute to its development
Code that is as understandable as plain English
Suitability for everyday tasks, allowing for short development times
'''
from src.config import LOCALHOST, SERVER_PORT
from socketio import Client, AsyncClient
import asyncio
import pickle
from threading import Thread

class Gadget(Client, Thread):
    def __init__(self, config: dict, hostname='localhost', port=8000, **kwargs):
        Client.__init__(self)
        Thread.__init__(self,
            target=self._connect,
            args=(LOCALHOST, SERVER_PORT))
        self.name = config.get('name','')
        self.namespace = f"/{self.name}"
        self.hostname=config.get('hostname',hostname)
        self.port = config.get('port',port)
        self.__dict__.update({'config':config})
        self.__dict__.update(**kwargs)
        self.message = Message
        pass
    
    # def emit(self, sid, data)
    def connect(self, hostname, port, header='http', namespaces=None):
        # self.connected = True
        Thread.start(self)
    
    def _connect(self, hostname, port, header='http'):
        print(f"{self.name} connecting to {header}://{hostname}:{port}/{self.name}")
        hostname='127.0.0.1'
        Client.connect(self,
            f"{header}://{hostname}:{port}",
            namespaces=["/",self.namespace],
            wait=False,
            wait_timeout=1)
        Client.wait(self)
    
    def emit(self, event, data, to=None):
        super().emit(event, pickle.dumps(data),
                     namespace=self.namespace)
    
    def pack_msg(self,func,**msg_kwargs):
        return self.message(**msg_kwargs)
    def unpack_msg(self,func,msg):
        return msg.__dict__
    
    def disconnect(self) -> None:
        if self.connected:
            self.join()
            
            super().disconnect()
            

class Phone(Gadget):
    def __init_(self, **kwargs):
        super.__init__(**kwargs)

class GamePad(Gadget):
    def __init_(self, **kwargs):
        super.__init__(**kwargs)



class Message(object):
    def __init__(self, *args, **kwargs):
        self.__dict__.update(**kwargs)
