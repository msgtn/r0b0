'''
The base Gadget class
'''
from r0b0 import logging
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.utils.loaders import decode_msg, encode_msg

from socketio import Client, ClientNamespace
from threading import Thread
import urllib3
urllib3.disable_warnings()
import ssl

ssl._create_default_https_context = ssl._create_unverified_context

HEADER = 'https'
EVENTS = []

class Message(object):
    """Base class for representing a message
    
    A message encapsulates the packets that is sent through socket connections.
    Gadgets that require more processing to parse/produce messages should 
    create their own message object that subclasses this
    """
    def __init__(self, *args, **kwargs):
        self.__dict__.update(**kwargs)

        # TODO - this is a kludge
        # Set solf dictionary to all information in the packet
        if kwargs.get('data',False):
            self.__dict__.update(**kwargs['data'])

class Gadget(Client, Thread):
    """Class representing a gadget

    More info...
    Even more info...

    Attributes:
        name: The name of the gadget
        namespace: The namespace of the gadget for socket connections
        config: The configuration dictionary that defines the gadget
        hostname: The hostname that the gadget should connect to
        port: The port that the gadget should connect to
        message: The Message type
    """
    def __init__(self, config: dict = {'type':'Gadget','name':'gadget'}, **kwargs):
        Client.__init__(self,           
            ssl_verify=False,            
            )
        Thread.__init__(self,
            target=self._connect_thread,
        )
        self.name = config.get('name','')
        self.namespace = f'/{config.get("namespace",self.name)}'

        self.__dict__.update({'config':config})
        self.__dict__.update(**kwargs)
        self.config = config
        self.hostname = self.config.get('hostname',LOCALHOST)
        self.port = self.config.get('port',SERVER_PORT)
        self.message = Message
    
    def _connect_thread(self,) -> None:
        url = f'{HEADER}://{self.hostname}:{self.port}{self.namespace}'
        logging.debug(f"{self.name} connecting to {url}")
        Client.connect(self,
            url=url,
            namespaces=[self.namespace,"/"],
            wait_timeout=2,
            )
        Client.wait(self)
        
    def handle_events(self,EVENTS):
        for _event in EVENTS:
            self.on(_event,
                handler=getattr(
                    self,
                    f'{_event}_event'
                ),
                namespace=self.namespace
            )
    
    @encode_msg
    def emit(self, event, data, **kwargs) -> dict:
        """
        Emits a message

        More info

        Args:
            event: The event to emit

        Returns:

        Raises: 
        """
        # overwrite defaults
        data.update(dict(
            event=data.get('event',event),
            id=data.get('id',self.sid)
        ))
        kwargs.update(dict(
            event=event,
            data=data
        ))
        
        # TODO - kludge to avoid BadNamespaceError if trying to emit before the gadget is connected
        # 
        try:
            Client.emit(self, **kwargs)
        except:
            pass
        return kwargs
    
    def assign_handlers(self, events_to_handle: list) -> None:
        for event in events_to_handle:
            self.on(event,
                handler=getattr(self, f'{event}_event'),
                namespace=self.namespace)
            pass    
        
    def unassigned_handler(self, data):
        logging.debug(f'{self.name} received unhandled event with data {data}')
    
    def disconnect(self) -> None:
        if self.connected: Client.disconnect(self)
        # if self.is_alive(): self.join()   

def init_gadget(gadget_type=Gadget, *args, **kwargs) -> Gadget:   
    return gadget_type(*args, **kwargs)

if __name__=="__main__":
    gadget = init_gadget()