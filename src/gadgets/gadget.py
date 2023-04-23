'''
GVR's design goals for Python
An easy and intuitive language just as powerful as major competitors
Open source, so anyone can contribute to its development
Code that is as understandable as plain English
Suitability for everyday tasks, allowing for short development times
'''
from src.config import LOCALHOST, SERVER_PORT
from src.utils.loaders import load_pickle, dump_pickle
from src import logging

from socketio import Client, AsyncClient, ClientNamespace
import asyncio
import pickle
from threading import Thread
import urllib3
urllib3.disable_warnings()
# import logging
# logging.basicConfig(level=logging.INFO)


class Message(object):
    def __init__(self, *args, **kwargs):
        # print(self.__dict__, type(self.__dict__), args, kwargs)
        self.__dict__.update(**kwargs)

class Gadget(Client, Thread):
    def __init__(self, config: dict, hostname=LOCALHOST, port=SERVER_PORT, **kwargs):
        Client.__init__(self,           
            ssl_verify=False,            
            )
        Thread.__init__(self,
            target=self._connect,
            # target=Client.wait
            # args=(LOCALHOST, SERVER_PORT)
        )
        self.name = config.get('name','')
        self.namespace = f"/{config.get('namespace',self.name)}"
        # self.namespace = [f"/{config.get('namespace',self.name)}"]
        # self.hostname = config.get('hostname',hostname)
        # self.port = config.get('port',port)
        self.hostname, self.port = hostname, port
        self.__dict__.update({'config':config})
        self.__dict__.update(**kwargs)
        self.message = Message
        # self.power_on, self.power_off = Thread.start, Thread.join
        # self._connect()
    
    # def gadget_connect(self, ):
    #     Thread.start(self)
    
    
    def _connect(self,):
        hostname=self.hostname
        port=self.port
        header='https'
        logging.debug(f"{self.name} connecting to {header}://{hostname}:{port}/{self.namespace}")
        Client.connect(self,
            url=f"{header}://{hostname}:{port}",
            # "https://r0b0.ngrok.io/",
            namespaces=[self.namespace,"/"],
            # wait=True,
            wait_timeout=1,
            )
        Client.wait(self)
    
    @dump_pickle
    # @load_pickle
    def emit(self, event, data, **kwargs):
        # data.update(dict(event=event))
        data.update(dict(event=data.get('event',event)))
        data.update(dict(id=data.get('id',self.sid)))
        # logging.debug(data)
        # logging.debug(self)
        # logging.debug(self.namespaces)
        # breakpoint() # TODO - self does not have any connected namespaces
        Client.emit(self,
            event,
            data,
            **kwargs)
        
    def pack_msg(self,func,**msg_kwargs):
        return self.message(**msg_kwargs)
    def unpack_msg(self,func,msg):
        return msg.__dict__
    
    def disconnect(self) -> None:
        if self.connected: Client.disconnect(self)
        # if self.is_alive(): self.join()   

class GamePad(Gadget):
    def __init_(self, **kwargs):
        super.__init__(**kwargs)
        
def init_gadget(gadget_type=Gadget, *args, **kwargs):
    return gadget_type(*args, **kwargs)

if __name__=="__main__":
    gadget = init_gadget()