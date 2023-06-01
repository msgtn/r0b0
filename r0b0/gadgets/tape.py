import sched
from r0b0.config import CSR_PEM, KEY_PEM, LOCALHOST, SERVER_PORT, TAPES_DIR
from .gadget import Gadget, Message

import os
import json
from collections import OrderedDict
from socketio import ClientNamespace
import numpy as np
from threading import Thread
from flask import render_template
import time
from socketio import Client
import pickle
import logging
logging.basicConfig(
    filename='phone.log',
    encoding='utf-8',
    level=logging.DEBUG,
)

class Tape(Gadget):
    '''
    A tape of recorded socket events
    '''
    def __init__(self,name='tmp',hostname=LOCALHOST,port=SERVER_PORT,**kwargs):
        Gadget.__init__(self,
            config={'name':name},
            hostname=hostname,
            port=port)
        self.filename = str( TAPES_DIR / f"{name}.json")
        self.tape = []
        self.sched = sched.scheduler(
            timefunc=time.time,
            delayfunc=time.sleep,
        )
        self.playing_thread = Thread(
            target=self._play,
        )
        
    def start(self):
        self.namespace = self.tape[0].get('namespace','')
        logging.debug(f"Tape namespace: {self.namespace}")
        # connect to server
        Gadget.start(self)
        logging.debug(self)
        # while len(self.namespaces)==0:
        while self.namespaces.get(self.namespace,None) is None:
            # continue
            logging.debug("Waiting for client connect")
            
        logging.debug(self.namespaces)
        self.playing_thread.start()
        
    def _play(self):
        t_last = 0
        for f,frame in enumerate(self.get_frame()):
            self.emit(**frame)
            # print(frame)
            time.sleep(frame['data']['time']-t_last)
            t_last = frame['data']['time']
            
    def _normalize_time(self):
        t_0 = self.tape[0]['data']['time']
        for f,frame in enumerate(self.tape):
            self.tape[f]['data']['time'] = (frame['data']['time'] -t_0)/10e3
            
    def open(self,tape_name,open_mode='r'):
        with open(tape_name,open_mode) as tape_file:
            return json.loads(''.join(tape_file.readlines()))
        
    @classmethod
    def load(self,name):
        filename = str( TAPES_DIR / f"{name}.json")
        # assert os.path.exists(filename), "No"
        if os.path.exists(filename):   
            tape = Tape(name)
            tape.tape = tape.open(filename)
            tape._normalize_time()
            return tape
        else:
            logging.warning(f"Tape '{name}' could not be loaded, does it exist in ./tapes/ ?")
            return None
            
    def write(self,frame):
        # TODO - check that the frame is valid, e.g. has event name and time is after last time
        self.tape.append(frame)
        
    def save(self):
        assert self.name is not 'tmp', "Name for tape not defined, can't save"
        assert len(self.tape)>0, f"Empty tape {self.name}, not saving"
        with open(self.filename,'w') as tape_file:
            tape_file.write(json.dumps(self.tape,indent=4))
        
    def get_frame(self,in_idx=0,out_idx=-1):
        # TODO - check if hit end of tape
        assert abs(in_idx)<len(self.tape) and abs(out_idx)<len(self.tape), "Indices longer than tape"
        subtape = self.tape[in_idx:out_idx]
        for f,frame in enumerate(subtape):
            print(frame['data']['time'])
            # yield [frame, f==(len(subtape)-1)]
            yield frame
        
    # def play(self):
    #     t_last = self.tape[0]['time']
    #     for frame in self.tape:
    #         self.host.emit(
    #             event=frame['event'],
    #             data=frame['data'],
    #         )
    #         t_event = frame['time']
    #         while (time.time()-t_last)<(t_event-t_last):
    #             continue
    #         t_last = t_event
    #     self.join()
    #     pass
    