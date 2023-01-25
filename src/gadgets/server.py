from src.config import \
    GADGETS_DIR, STATIC_DIR, PUBLIC_DIR, \
    LOCALHOST, SERVER_PORT, \
    CSR_PEM, KEY_PEM

# import eventlet
# from eventlet.green.OpenSSL import SSL
from aiohttp import web
# import socketio
from socketio import AsyncServer, Server, Namespace
import pickle
from functools import partial
from threading import Thread

from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit
from flask_cors import CORS
    
class Host(Thread, SocketIO):        
# class Host(SocketIO, Thread):        
    def __init__(self, hostname=LOCALHOST, port=SERVER_PORT,**kwargs):
        self.app = app = Flask(__name__)
        CORS(self.app)
        self.hostname = hostname
        self.port = port
        SocketIO.__init__(self,
            self.app,
            cors_allowed_origins=[
                "*",
                "https://r0b0.ngrok.io",
                f"https://{LOCALHOST}:{SERVER_PORT}",
            ],
            **kwargs
            )
        Thread.__init__(self,
            # TODO - in order for this to work, must subclass Thread before SocketIO
            # probably because they both have run() functions
            target = SocketIO.run,
            args=(self, self.app),
            kwargs={
                'host':self.hostname,
                'port':self.port,
                'certfile':CSR_PEM,
                'keyfile':KEY_PEM,
            })
        
        SocketIO.on_event(
            self,
            'device_motion',
            handler=self.device_motion)
        
        @app.route('/')
        def index():
            return render_template('index.html')
        @app.route('/reset',methods=["GET","POST"])
        def reset():
            print('reset')
            return "<p>Hello, World!</p>"
        
        
    def add_route(self, route, route_func):
        # @self.app.
        self.app.add_url_rule(route, view_rule=route_func)
        
        
    
    def device_motion(self,data, **kwargs):
        print('device_motion',data, kwargs)
        # self.emit('python',{'data':42})
        
    def connect_event(self,sid, environ, auth,):
        print(f"Server connected to {sid}")
        
class _Host(Server, Thread):        
    def __init__(self, hostname=LOCALHOST, port=SERVER_PORT, **kwargs):
        Server.__init__(self,
            async_mode='eventlet',
            **kwargs
        )
        self.static_files = {
            '/':str(PUBLIC_DIR / 'index.html'),
            '/controller.js':str(PUBLIC_DIR / 'controller.js')
            # '/':'./src/gadgets/public'
        }
        self.hostname = hostname
        self.port = port
        self.on('connect',self.connect_event)
        Thread.__init__(self,
            target=self._start)
        
    def connect_event(self,sid, environ, auth,):
        print(f"Server connected to {sid}")
        
    def _start(self,):
        self.wsgi_app = socketio.WSGIApp(
            self, 
            static_files=self.static_files)
        self.ws_server = eventlet.listen(
            (self.hostname, self.port))
        
        eventlet.wsgi.server(
            self.ws_server, self.wsgi_app)


class HostNamespace(Namespace):
    def on_connect(self, sid, environ):
        super().on_connect(self, sid, environ)
            
    def on_disconnect(self, sid):
        super().on_disconnect(self, sid)
        
    def on_robot(self, sid, data):
        pass
        
    
async def index(request):
    with open(str(PUBLIC_DIR / 'index.html')) as f:
        return web.Response(text=f.read(), content_type='text/html')                                        

# sio = Host()
# sio.register_namespace(HostNamespace('/robot'))

def main_async():
    sio = Host()
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
    Host().start()
    # app = Flask(__name__)
    # FlaskHost(LOCALHOST, SERVER_PORT).start()