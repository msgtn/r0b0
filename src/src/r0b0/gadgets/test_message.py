import unittest
import pickle
import mido
from .gadget import Message
from .midi_controller import MIDIMessage


class MessageTest(unittest.TestCase):
    def test_json_dumps(self):
        msg = Message(event="test", value=60)
        # json.dumps(msg)


class MIDIMessageTest(unittest.TestCase):
    def test_from_mido(self):
        mido_msg = mido.Message("note_on", note=60)
        # midi_msg = MIDIMessage.from_mido(mido_msg)
        # assert midi_msg.event=='midi_on'
        # assert midi_msg.value==60
