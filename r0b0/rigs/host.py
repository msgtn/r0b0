SOCKET_ADDR = "https://r0b0.ngrok.io"
# SOCKET_ADDR = "https://a6f7039dadaa.ngrok.app"


# SOCKET_ADDR = "https://104e-32-221-140-83.ngrok-free.app"

import glob
# import logging

from r0b0.config import \
    ROOT_DIR, TAPES_DIR, \
    GADGETS_DIR, STATIC_DIR, PUBLIC_DIR, \
    LOCALHOST, SERVER_PORT, \
    CSR_PEM, KEY_PEM, BROWSER_DIR
from r0b0.utils.loaders import decode_msg,encode_msg
from r0b0.gadgets import Tape
from r0b0 import logging, get_timestamp

import os
from aiohttp import web
from socketio import AsyncServer, Server, Namespace
import pickle
from functools import partial, partialmethod, wraps
from threading import Thread
from copy import copy, deepcopy

import sched
from collections import OrderedDict

from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import time
import json
# TODO - try to remove dependency of having to import Thread first
# to call the correct *.run()


PLAYER_EVENTS = [
    'load','play','record',
]

class Host(Thread, SocketIO):      
    def __init__(self, hostname=LOCALHOST, port=SERVER_PORT, **kwargs):
        self.app = app = Flask(
            __name__,
            # TODO - was trying to direct templates to a different folder
            # template_folder=str(BROWSER_DIR / 'templates'),
            # template_folder=str(BROWSER_DIR),
            )
        CORS(self.app)
        self.hostname = hostname
        self.port = port
        # print(f'host port {port}')
        SocketIO.__init__(self,
            self.app,
            cors_allowed_origins=[
                "*",
                SOCKET_ADDR,
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
        
        SocketIO.on_event(self,
            'add_url',
            self.add_url,
        )
        SocketIO.on_event(self,
            'add_emit',
            self.add_emit,)
        
        self._webrtc_setup()
        self._player_setup()
        
        # self.power_on, self.power_off = self.start, self.join
    
    # @encode_msg
    def emit(self, *args, **kwargs):
        logging.debug(args)
        logging.debug(kwargs)
        
        # if the event is a player-related event,
        # handle it internally
        if kwargs['event'] in PLAYER_EVENTS:
            getattr(self,f"on_{kwargs['event']}")(*args, **kwargs)
        else:
            SocketIO.emit(self, *args, **kwargs)

    @decode_msg
    def add_url(self, data):
        route_func = lambda: render_template(data['url'])
        route_func.__name__ = \
            f"route_{data['url'].split('.')[0]}"
        self.app.add_url_rule(
            data['route'],
            view_func=route_func)
    @decode_msg
    def add_emit(self,data):
        logging.debug('add_emit')
        logging.debug(data)
        event = data['event']
        def _emit_record(s,d):
            logging.debug(s)
            logging.debug(d)
            
            self.emit(
                event=d['event'],
                data=d,
                **data['kwargs'] # namespace arg from the .yaml that defined it
            )
            id_event = f"{d['id']}_{d['event']}"
            tape = self.tapes.get(id_event,None)
            if tape is not None:
            # if id_event in self.tapes.keys():
                # record time in millis
                d.update({
                    'time':int(time.time()*10e3),
                    })
                tape.write({**{
                    'event':data['event'],
                    'data':d},
                    **data['kwargs'],
                })
        self.server.on(
            data['event'],
            _emit_record,
            # TODO - figure out how to have this on device-specific namespace
            namespace="/"
            # **data['kwargs']
        )
            
    # metaphor - VCR player
    def _player_setup(self):
        self.tapes = OrderedDict()
        for player_event in PLAYER_EVENTS:
            SocketIO.on_event(self,
                player_event,
                getattr(self,f"on_{player_event}"))   
        self.app.add_url_rule(
            '/tapes',
            view_func=self.get_tapes,
        )

    def get_tapes(self):
        # tapes = send_from_directory()
        return sorted(os.listdir(TAPES_DIR))
    
    def on_load(self, data):
        logging.debug(data)
        tape_name = data['tape_name']
        if tape_name in self.tapes.keys():
            return self.tapes[tape_name]
        tape = Tape.load(tape_name)
        if tape:
            self.tapes.update({
                tape_name:tape
            })
        return tape
    
    def on_record(self, data):
        '''
        data = {
            record: true(start)/false(stop),
            event: str,
        }
        '''
        id_event = f"{data['id']}_{data['event']}"
        
        # data['record'] is boolean
        if data['record']:
            # start recording, make a new Tape
            self.tapes.update({
                id_event:Tape(f"{get_timestamp()}_{data['event']}")
            })
        else: 
            # stop recording, get the Tape and save
            tape = self.tapes.pop(id_event,None)
            if tape:
                tape.save()
        logging.debug(self.tapes)
        
    @decode_msg
    def on_play(self, data, **kwargs):
        logging.debug(data)
        if 'msg' in data:
            data.update(data['msg'].__dict__)
        tape = self.tapes.get(data['tape_name'],None)
        if tape is None and 'msg' in data:
            # tape = self.tapes.getattr(data['msg'],'tape_name',None)
            tape = self.tapes.get(
                getattr(data['msg'],'tape_name',None),None
            )
        logging.debug(f"tape {tape}")
            
        if tape is not None:
            tape.play()
        else:
            # try to load tape
            tape = self.on_load(data)
            if tape is not None:
                tape.play()
            # could not load tape
            else:
                logging.warning(f"No tape {data['tape_name']}, cannot play")
                
    # TODO - wrapper play function for CLI usage
    def play(self, tape_name):
        self.on_load({'tape_name':tape_name})
        self.on_play({'tape_name':tape_name})
        

    # no metaphor for this one
    def _webrtc_setup(self):
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
    host = Host()
    host.start()
    breakpoint()