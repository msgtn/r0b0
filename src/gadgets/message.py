
class Message(object):
	def __init__(self):
		pass
		
	def pack(self):
	    pass
	
	def unpack(self):
	    pass
		
import mido
class MIDIMessage(Message, mido.Message):
	def __init__(self):
	    super().__init__()
	
class MotorMessage(Message):
    def __init__(self):
	    super().__init__()

from scipy.rotation import Rotation
class OrientationMessage(Message):
	def __init__(self):
	    super().__init__()
	
	
midimsg = MIDIMessage('note_on', note=60)