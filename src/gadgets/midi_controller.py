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
        
    def midi_callback(self, mido_msg):
        midi_msg = MIDIMessage.from_mido(mido_msg)
        if self.connected:
            if self.echo:
                Gadget.emit(
                    self,
                    midi_msg.event,
                    midi_msg)
                    
    def disconnect(self):
        super().disconnect()
        self.midi_port.close()

class MIDINamespace(Namespace):
    def on_midi_cc(self, sid, data):
        pass

class MIDIMessage(Message):
    def __init__(self, **kwargs):
        Message.__init__(self, **kwargs)
        
    @classmethod
    def from_mido(self, mido_msg: mido.Message):
        mido_dict = {}
        # update instances with basic fields of types (str,bool,int,float)
        mido_dict.update({
            k:v for k,v in mido_msg.__dict__.items() if isinstance(
                v,(str,bool,int,float))})
        value = mido_dict.get('note',None) or mido_msg.value
        event = EVENT_TABLE.get(mido_msg.type,mido_msg.type)
        mido_dict.update(dict(
            event=event,
            value=value,
        ))
        return MIDIMessage(**mido_dict)
        