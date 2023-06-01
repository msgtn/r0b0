from .gadget import Gadget, Message
from r0b0.utils.loaders import load_pickle
from r0b0 import logging

from pymongo import MongoClient

class TapeDeck(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init_(self, config, **kwargs)
        # TODO - check if mongod service is running
        # mongod --config /opt/homebrew/etc/mongod.conf
        self.mongo_clients = [MongoClient(**config['clientUrls'])]
        self.tapes = {}
        
    def _player_setup(self):
        self.tapes = {}
        player_events = [
            'load','play','record',
        ]
        for player_event in player_events:
            self.on_event(
                player_event,
                getattr(self,f"on_{player_event}"))
        
        # self.on('')
    # TODO - load local tapes
    
    # TODO - handle playback
    
    @load_pickle
    def on_load(self, data):
        logging.debug(data)
        tape_name = data['tape_name']
        # self.tapes.update({
        #     tape_name:Tape.load(tape_name)
        # })
        
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
        # TODO - make this a thread
        if tape:
            playing = True
            while playing:
                frame, playing = tape.get_frame()
                self.emit(**frame)

    