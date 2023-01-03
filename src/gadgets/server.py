import eventlet
from src.config import \
    GADGETS_DIR, STATIC_DIR, PUBLIC_DIR, \
    LOCALHOST, SERVER_PORT
from aiohttp import web
import socketio
from socketio import AsyncServer, Server, Namespace
import pickle
from functools import partial

# from multiprocessing import Process
from threading import Thread
# register(sid_gadget_from, sid_gadget_to, func_translate)


# def translation_cable(func, sender_message, receiver_message):
#     def inner_func(*args, **kwargs):
#         return receiver_message(partial(func, sender_message=sender_message))
#     return inner_func

# @translation_cable
# # make these functions as simple and small as possible
# # because many of them will get written
# # @to_motor
# # @from_midi
# def from_cable(func):
#     pickle.loads()

class Cable(Server, Thread):        
    def __init__(self, hostname=LOCALHOST, port=SERVER_PORT, **kwargs):
        Server.__init__(
            self,
            async_mode='eventlet',
            **kwargs
        )
        self.gadgets = []
        self.static_files = {
            '/':{
                'content_type': 'text/html',
                'filename':'index.html'
                }    
        }
        self.on('connect',self.connect_event)
        # self.on('midi_cc',handler=self.print_event,namespace='/opz')
        # self.on('*',handler=self.print_event)
        self.app = socketio.WSGIApp(
            self, self.static_files)
        self.ws_server = eventlet.listen((hostname, port))
        Thread.__init__(self,
            target=self._start)
        
    # def print_event(self,event, data):
        # print(f'opz {data}')
        
    def connect_event(self,sid, environ, auth,):
        print(f"Connected to {sid}")
        print(environ,auth)
        
    def _start(self,):
        eventlet.wsgi.server(
            self.ws_server, self.app)


class CableNamespace(Namespace):
    def on_connect(self, sid, environ):
        super().on_connect(self, sid, environ)
            
    def on_disconnect(self, sid):
        super().on_disconnect(self, sid)
        
    def on_robot(self, sid, data):
        pass
        
    
async def index(request):
    with open(str(PUBLIC_DIR / 'index.html')) as f:
        return web.Response(text=f.read(), content_type='text/html')                                        

# sio = Cable()
# sio.register_namespace(CableNamespace('/robot'))

def main_async():
    sio = Cable()
    app = web.Application()
    app.router.add_static(str(STATIC_DIR), str(PUBLIC_DIR))
    app.router.add_get(str(GADGETS_DIR),index)
    sio.attach(app)

    web.run_app(app,
        host=LOCALHOST,
        port=SERVER_PORT)
 
def start_server(sio):
    app = socketio.WSGIApp(sio, sio.static_files)
    eventlet.wsgi.server(
        sio.ws_server, app)
    
if __name__=="__main__":
    # start_server(LOCALHOST, SERVER_PORT)
    # sio.start(LOCALHOST, SERVER_PORT)
    
    Cable().start()