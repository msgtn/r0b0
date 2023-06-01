from .gadget import Gadget, Message
from r0b0.utils.loaders import load_pickle
from r0b0 import logging

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
MIDO_ARGS = [
    'type','channel',
    'note','velocity',
    'value',
    'program',
    'pitch',
    'control'
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
        self.on('midi',
            handler=self.on_midi,
            namespace=self.namespace)
        
    def midi_callback(self, mido_msg):
        midi_msg = MIDIMessage.from_mido(mido_msg)
        if midi_msg.type in MUTE_EVENTS: return
        # logging.debug(midi_msg.type)
        # logging.debug(midi_msg.__dict__)
        # print('midi_msg',midi_msg)
        # print(midi_msg,self.connected,self.echo,midi_msg.event)
        if self.connected:
            if self.echo:
                # print(midi_msg,midi_msg.event, )
                # Gadget.emit(
                #     self,
                self.emit(
                    event=midi_msg.event,
                    data={'event':midi_msg.event,'msg':midi_msg},
                    namespace=self.namespace)
                    
    @load_pickle
    def on_midi(self,data):
        logging.debug(data)
        self.midi_port.send(
            # TODO - update to self-defined MIDIMessage(**data)
            data['msg']
        )
                   
    def disconnect(self):
        super().disconnect()
        self.midi_port.close()

class MIDINamespace(Namespace):
    def on_midi_cc(self, sid, data):
        pass

# TODO - subclass mido's messages
class MIDIMessage(Message, MidoMessage):
    def __init__(self, **kwargs):
        Message.__init__(self, **kwargs)
        mido_kwargs = {k:v for k,v in kwargs.items() if k in MIDO_ARGS}
        # logging.debug(kwargs)
        # logging.debug(mido_kwargs)
        # print(mido_kwargs)
        MidoMessage.__init__(self,**mido_kwargs)
        
    @classmethod
    def from_mido(self, mido_msg: mido.Message, **kwargs):
        self = mido_msg
        mido_dict = {}
        # update instances with basic fields of types (str,bool,int,float)
        mido_dict.update({
            k:v for k,v in mido_msg.__dict__.items() if isinstance(
                v,(str,bool,int,float))})
        
        return MIDIMessage(
            event=EVENT_TABLE.get(
                mido_msg.type,
                'midi_cc'),            
            msg=mido_msg,
            **mido_dict)
        