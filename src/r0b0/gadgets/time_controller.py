from enum import Enum
from .gadget import Gadget, Message, logging
from r0b0.utils.loaders import decode_msg


# MODES = ['idle','stopwatch','timer']
class TimeMode(Enum):
    IDLE = idle = 1
    STOPWATCH = stopwatch = 2
    TIMER = timer = 3

EVENTS = [
    'set_mode',
]

class TimeController(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self,
            config,
            # request_timeout=0.1,
            # *args,
            **kwargs)
        self.handle_events(EVENTS)
        self.mode = TimeMode.IDLE

    @decode_msg
    def set_mode_event(self, data):
        msg = data['msg']
        self.mode = getattr(TimeMode, msg.mode.upper())

    # @encode_msg
    # def 
