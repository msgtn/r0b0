# from src import gadgets
# import src.gadgets
from src import gadgets as gadget_shelf
# from src.gadgets.server import start_server
from src.gadgets.server import Cable
from multiprocessing import Process

class Rig(object):
    def __init__(self, hostname='localhost', port=8080):
        self.gadgets = {}
        self.server = None
        self.hostname = hostname
        self.port = port
        self.power = False
    
    def add_gadget(self, gadget_name):
        config = loaders.load_config(gadget_name)
        GadgetObject = getattr(gadget_shelf, config['type'])
        self.gadgets.update({
            config['name']:getattr(gadgets, config['type'])(
                config
            )
        })
    
    def add_server(self, hostname, port):
        self.server = Cable(hostname,port)
        
    def power_on(self,):
        self.server.start()
        for gadget in self.gadgets:
            gadget.start()
            
        self.power = True
            
    def power_off(self,**kwargs):
        self.server.join()
        for gadget in self.gadgets:
            gadget.join()
        self.power = False
        
    def add_message(self, from_gadget, to_gadget, func):
        # @self.gadgets[to_gadget].pack_msg
        # @self.gadgets[from_gadget].unpack_msg
        def func_emit(sid, data):
            return self.gadgets['to_gadget'].message(**func(data))
        self.server.on(
            'event_name',
            # self.gadgets['to_gadget'].message(**func()),
            # self.server.emit(
            #     'func_type_name',
            func_emit,
            #     namespace=f"/{to_gadget}"
            # ),
            namespace=f"/{from_gadget}",
        )
        
    # def start_server(self, ):
    #     return Process(
    #         target=start_server,
    #         args=(self.hostname,self.port),
    #     ), hostname, port
        
    # def start_gadget(self, gadget):
    #     return Process(
    #         target=gadget.connect,
    #         args=(self.hostname, self.port)
    #     )
        
    # def power_on(self,):
    #     self.server_process = self.start_server()
    #     self.gadget_processes = {
    #         gadget.name:self.start_gadget(gadget) for gadget in self.gadgets
    #     }
    #     self.power = True

    # def power_off(self):
    #     self.server_process.join()
    #     [p.join() for p in list(self.gadget_processes.values())]
    #     self.power = False        
    
    