# from src import gadgets
# import src.gadgets
from src import gadgets as gadget_shelf
from src.config import LOCALHOST, SERVER_PORT
from src.utils import loaders
# from src.rigs import logging
# from src.gadgets.rig import start_server
from src.rigs.server import Host
from src.messages import msg_funcs
from multiprocessing import Process
import pickle
from  src import logging
# logging = logging.getLogger(__name__)

class Rig(Host):
    def __init__(self, hostname=LOCALHOST, port=SERVER_PORT, **kwargs):
        Host.__init__(self, hostname, port, **kwargs)
        self.gadgets = {}
        self.hostname = hostname
        self.port = port
        self.power = False
    
    def add_gadget(self, gadget_name):
        config = loaders.load_gadget(gadget_name)
        gadget = getattr(gadget_shelf, config['type'], None)
        assert gadget is not None, f"Gadget type {config['type']} does not exist"
        gadget = gadget(config)
        self.gadgets.update({
            gadget_name:gadget
        })
        return gadget
    
    def _get_gadget_namespace(self, gadget):
        return self.gadgets.get(gadget).namespace
        
    def add_message(self, tx_gadget, rx_gadget, msg_func):
        logging.debug('add_message',tx_gadget, rx_gadget, msg_func)
        
        tx_namespace, rx_namespace = map(
            self._get_gadget_namespace,
            [tx_gadget, rx_gadget])
        print( tx_namespace, rx_namespace)
        msg_func = getattr(msg_funcs,msg_func)
        def func_emit(data):
            # if not isinstance(data,dict): data = pickle.loads(data)
            emit_data = self.gadgets[rx_gadget].message(
                **msg_func(data))
            print(data)
            self.emit(
                event=emit_data.event,
                data={'event':emit_data.event,'msg':pickle.dumps(emit_data)},
                to=None,
                namespace=rx_namespace
            )
        # print('tx',tx_namespace)
        self.on_event(
            msg_func()['event'],
            handler=func_emit,
            namespace=tx_namespace
        )
        
        
    def power_on(self,):
        # breakpoint()
        self.start()
        print(self.gadgets.values())
        [g.start() for g in self.gadgets.values()]
        self.power = True
            
    def power_off(self,*args,**kwargs):
        assert self.power, "Rig not powered on"
        self.disconnect()
        [g.disconnect() for g in self.gadgets.values()]
        self.power = False
        