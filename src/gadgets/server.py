from src.config import \
    GADGETS_DIR, STATIC_DIR, PUBLIC_DIR, \
    LOCALHOST, SERVER_PORT, \
    CSR_PEM, KEY_PEM

# import eventlet
# from eventlet.green.OpenSSL import SSL
from aiohttp import web
# import socketio
from socketio import AsyncServer, Server, Namespace
from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import pickle
from functools import partial
from threading import Thread
    
class FlaskHost(Thread, SocketIO):        
    def __init__(self, hostname, port,**kwargs):
        self.app = app = Flask(__name__)
        CORS(self.app)
        self.hostname = hostname
        self.port = port
        SocketIO.__init__(self,
            self.app,
            # cors_allowed_origins=["*","https://localhost:8080"],
            cors_allowed_origins=[
                "*",
                "https://r0b0t.ngrok.io",
                "https://localhost",
            ])
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
        
        
        SocketIO.on_event(self,'position',handler=self.position_message)
        
        @app.route('/')
        def index():
            return render_template('index.html')
        @app.route('/reset',methods=["GET","POST"])
        def reset():
            print('reset')
            # return 200
            return "<p>Hello, World!</p>"
            # return render_template('index.html')
        
    
    def position_message(self,data):
        print('position',data)
        # self.emit('updateRecInd',{'data':42})
        
        
    def connect_event(self,sid, environ, auth,):
        print(f"Server connected to {sid}")
        
    def _start(self,):
        SocketIO.run(self,
            app=self.app,
            # debug=True,
            host=LOCALHOST,
            # host='0.0.0.0',
            # port=SERVER_PORT,
            port=9000,
            # certfile=SERVER_CRT,
            # keyfile=SERVER_KEY,
            # certfile=str(CSR_PEM),
            # keyfile=str(KEY_PEM),
            # ssl_context=(SERVER_CRT,SERVER_KEY),
            ssl_context=(str(CSR_PEM),str(KEY_PEM)),
            )

class Host(Server, Thread):        
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
    # Host().start()
    app = Flask(__name__)
    FlaskHost(LOCALHOST, SERVER_PORT).start()