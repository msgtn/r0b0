from src.config import \
    GADGETS_DIR, STATIC_DIR, PUBLIC_DIR, \
    LOCALHOST, SERVER_PORT, \
    CSR_PEM, KEY_PEM

from aiohttp import web
from socketio import AsyncServer, Server, Namespace
import pickle
from functools import partial, partialmethod
from threading import Thread

from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit
from flask_cors import CORS
    
# TODO - try to remove dependency of having to import Thread first
# to call the correct *.run()
class Host(Thread, SocketIO):        
    def __init__(self, hostname=LOCALHOST, port=SERVER_PORT, **kwargs):
        self.app = app = Flask(__name__)
        CORS(self.app)
        self.hostname = hostname
        self.port = port
        SocketIO.__init__(self,
            self.app,
            cors_allowed_origins=[
                "*",
                "https://r0b0.ngrok.io",
                f"https://{self.hostname}:{self.port}",
            ],
            **kwargs
            )
        Thread.__init__(self,
            # TODO - as above, in order for this to work, must subclass Thread before SocketIO
            # because they both have run() functions
            target = SocketIO.run,
            args=(self, self.app),
            kwargs={
                'host':self.hostname,
                'port':self.port,
                'certfile':CSR_PEM,
                'keyfile':KEY_PEM,
            })
        
        self.broadcaster_id = None
        
        webrtc_events = [
            'broadcaster', 'watcher',
            'offer','answer',
            'candidate'
        ]
        for webrtc_event in webrtc_events:
            SocketIO.on_event(self,
                webrtc_event,
                getattr(self,webrtc_event))
        
        @app.route('/')
        def index():
            return render_template('index.html')
        @app.route('/broadcast')
        def broadcast():
            return render_template('broadcast.html')
        @app.route('/reset',methods=["GET","POST"])
        def reset():
            print('reset')
            return "<p>Hello, World!</p>"
        
    def broadcaster(self, sid):
        print('broadcaster', sid)
        self.broadcaster_id = sid
        SocketIO.emit(self,
            'broadcaster')
        
    def watcher(self,sid):
        if not self.broadcaster_id: return
        SocketIO.emit(self,
            'watcher',
            # {'sid':request.sid},
            request.sid,
            # args={'sid':request.sid},
            to=self.broadcaster_id,
            )
        print('watcher', sid)

    def offer(self,sid,msg, *args,**kwargs):
        # breakpoint()
        SocketIO.emit(self,
            'offer',
            (request.sid,msg),
            to=sid,
            )
    def answer(self,sid,msg):
        # TODO - handle max connections
        # print(sid,msg)
        SocketIO.emit(self,
            'answer',
            (request.sid,msg),
            to=sid,
            )
    def candidate(self,sid,msg):
        SocketIO.emit(self,
            'candidate',
            (request.sid,msg),
            to=sid,
            # args=msg,        
        )
        
    def device_motion(self,data):
        print(data)

    def add_route(self, route, route_func):
        self.app.add_url_rule(route, view_rule=route_func)
        
    def connect_event(self,sid, environ, auth,):
        print(f"Server connected to {sid}")

if __name__=="__main__":
    Host().start()
    # app = Flask(__name__)
    # FlaskHost(LOCALHOST, SERVER_PORT).start()