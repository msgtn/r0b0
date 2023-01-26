from src.config import \
    GADGETS_DIR, STATIC_DIR, PUBLIC_DIR, \
    LOCALHOST, SERVER_PORT, \
    CSR_PEM, KEY_PEM
from src.utils.data import load_pickle,dump_pickle

from aiohttp import web
from socketio import AsyncServer, Server, Namespace
import pickle
from functools import partial, partialmethod, wraps
from threading import Thread
from copy import copy, deepcopy

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
            
        SocketIO.on_event(self,
            'add_url',
            self.add_url,
        )
        SocketIO.on_event(self,
            'add_emit',
            self.add_emit,)
        
    @load_pickle
    def add_url(self, data):
        route_func = lambda: render_template(data['url'])
        route_func.__name__ = f"route_{data['url'].split('.')[0]}"
        self.app.add_url_rule(
            data['route'],
            view_func=route_func
        )
        
    @load_pickle
    def add_emit(self,data):
        self.server.on(
            data['event'],
            lambda s,d: self.emit(
                event=data['event'],
                data=d,
                **data['kwargs'],
            )
        )
        
    def broadcaster(self, sid):
        self.broadcaster_id = sid
        SocketIO.emit(self,
            'broadcaster')
    def watcher(self,sid):
        if not self.broadcaster_id: return
        SocketIO.emit(self,
            'watcher',
            request.sid,
            to=self.broadcaster_id,
            )
    def offer(self,sid,msg, *args,**kwargs):
        SocketIO.emit(self,
            'offer',
            (request.sid,msg),
            to=sid,
            )
    def answer(self,sid,msg):
        # TODO - handle max connections
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
        )

if __name__=="__main__":
    Host().start()