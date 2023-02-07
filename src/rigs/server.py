import glob
# import logging

from src.config import \
    ROOT_DIR, TAPES_DIR, \
    GADGETS_DIR, STATIC_DIR, PUBLIC_DIR, \
    LOCALHOST, SERVER_PORT, \
    CSR_PEM, KEY_PEM
from src.utils.loaders import load_pickle,dump_pickle
from src.gadgets import Tape
from src import logging

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
class Host(Thread, SocketIO):      
    def __init__(self, hostname=LOCALHOST, port=SERVER_PORT, **kwargs):
        self.app = app = Flask(__name__)
        CORS(self.app)
        self.hostname = hostname
        self.port = port
        # print(f'host port {port}')
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
    
    def emit(self, *args, **kwargs):
        logging.debug(args)
        logging.debug(kwargs)
        SocketIO.emit(self, *args, **kwargs)
    
    # metaphor - VCR player
    def _player_setup(self):
        self.tapes = OrderedDict()
        player_events = [
            'load','play','record',
        ]
        for player_event in player_events:
            SocketIO.on_event(self,
                player_event,
                getattr(self,f"on_{player_event}"))
    
    def on_load(self, data):
        logging.debug(data)
        tape_name = data['tapeName']
        tape = Tape.load(tape_name)
        if tape:
            self.tapes.update({
                tape_name:tape
            })
        
    def on_record(self, data):
        '''
        data = {record: true/false, event: str}
        '''
        id_event = f"{data['id']}_{data['event']}"
        if data['record']:
            # start recording, make a new Tape
            self.tapes.update({
                id_event:Tape(f"{time.strftime('%Y%m%d%H%M%S')}_{data['event']}")
            })
        else: 
            # stop recording, get the Tape and save
            tape = self.tapes.pop(id_event,None)
            if tape:
                tape.save()
        logging.debug(self.tapes)
        
    def on_play(self, data):
        tape = self.tapes.get(data['tape_name'],None)
        if tape is not None:
            # print(tape.get_frame())
            # f = tape.get_frame()
            # [print(_f) for _f in f]
            # frame, playing = tape.get_frame()
            # t_0 = frame['data']['time']
            # tape_time = lambda : time.time-(frame['data']['time']/10e3)
            
            # while playing:
            tape_sched = None
            
            for frame,_ in tape.get_frame():
                if tape_sched is None:
                    t_0 = frame['data']['time']
                    tape_sched = sched.scheduler(
                        # define time relative to first frame
                        # timefunc=lambda: time.time()-t_0/10e3,
                        timefunc=time.time,
                        delayfunc=time.sleep)
                    
                # frame['kwargs'] = {'namespace':'/blossom'}
                    
                # tape_sched.enterabs(
                tape_sched.enter(
                    # time=frame['data']['time']/10e3,
                    delay=frame['data']['time'],
                    priority=1,
                    action=self.emit,
                    kwargs=frame,
                )
                # frame, playing = tape.get_frame()
            tape_sched.run()            

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
            
    @load_pickle
    def add_url(self, data):
        route_func = lambda: render_template(data['url'])
        route_func.__name__ = \
            f"route_{data['url'].split('.')[0]}"
        # print(data,route_func.__name__)
        self.app.add_url_rule(
            data['route'],
            view_func=route_func)
    @load_pickle
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
                    'time':int(time.time()*1e3),
                    })
                tape.write({
                    'event':data['event'],
                    'data':d,
                    'kwargs':data['kwargs']
                })
        self.server.on(
            data['event'],
            _emit_record,
            # TODO - figure out how to have this on device-specific namespace
            namespace="/"
            # **data['kwargs']
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
    host = Host()
    host.start()
    breakpoint()