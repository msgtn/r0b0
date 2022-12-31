from .gadget import Gadget, Message

import mido
from mido.sockets import PortServer, connect
from mido.ports import BaseIOPort
from mido import Message as MidoMessage
import pickle
from socketio import Namespace

EVENT_TABLE = {
    'note_on': 'midi_on',
    'note_off': 'midi_off',
    'control_change': 'midi_cc'
}
MUTE_EVENTS = [
    'clock'
]

class MIDIController(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.midi_port = mido.open_ioport(
            config['port_name'],
            callback=self.midi_callback)
        self.echo = True
        [EVENT_TABLE.pop(mute_message,None) for mute_message in MUTE_EVENTS]
        self.message = MIDIMessage
        
    def midi_callback(self, message):
        # print(message)
        midi_event = EVENT_TABLE.get(message.type, None)
        if not midi_event: return
        if self.connected:
            if self.echo:
                print(message)
                Gadget.emit(self,midi_event, message)
                    
    # def connect(self, *args, **kwargs):
    #     Gadget.connect(self,*args,**kwargs)
    
    def disconnect(self):
        super().disconnect()
        self.midi_port.close()

class MIDINamespace(Namespace):
    def on_midi_cc(self, sid, data):
        pass
    
def from_midi(func):
    '''
    what am i trying to do here
    the decorator just does stuff around the function
    it cant go inside the function itself
    '''
    
    return func

class MIDIMessage(Message, MidoMessage):
    def __init__(self, **kwargs):
        Message.__init__(self, **kwargs)
        MidoMessage.__init__(self, **kwargs)
        