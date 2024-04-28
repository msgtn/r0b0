from enum import Enum
from .gadget import Gadget, Message, logging
from r0b0.utils.loaders import decode_msg
from timeit import default_timer
from threading import Thread
from functools import partial

# MODES = ['idle','stopwatch','timer']
class TimeMode(Enum):
    IDLE = idle = 1
    STOPWATCH = stopwatch = 2
    TIMER = timer = 3


EVENTS = [
    "set_mode",
]


class TimeController(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(
            self,
            config,
            # request_timeout=0.1,
            # *args,
            **kwargs
        )
        self.handle_events(EVENTS)
        self.mode = TimeMode.IDLE
        self.tick_thread = None

    @decode_msg
    def set_mode_event(self, data):
        msg = data["msg"]
        last_mode = self.mode
        self.mode = getattr(TimeMode, msg.mode.upper())
        # "debounce"
        if last_mode!=self.mode:
            print(self.mode)
            if self.mode==TimeMode.STOPWATCH:
                self.direction = -1
            elif self.mode==TimeMode.TIMER:
                self.direction = 1
            if self.tick_thread is not None:
                self.tick_thread.join()
                self.tick_thread = None
            self.tick_thread = Thread(
                target=partial(self._tick_thread, direction=self.direction)
            )
            self.tick_thread.start()


    # What else does this have to do
    # Timeout after mode is changed, after the last read velocity or mode change event
    # Send an enable event
    # When the motor moving on its own:
    # Send a tick event
    # either up or down
    # This should be a thread that is remade after every setting of the mode
    # Not here, but a cable will translate the tick to position events 

    def _tick_thread(self, direction=1, *args, **kwargs):
        print(f"Starting tick thread with {direction:}")
        last_time = default_timer()
        while self.direction==direction:
            if default_timer()-last_time >= 1.0:
                self._tick(direction, *args, **kwargs)
                last_time = default_timer()

    def _tick(self, direction=1):
        print(f"tick {direction}")
        self.emit(
            event="tick",
            data={
                "event":"tick",
                "msg": Message(event="tick", direction=direction)
            }
        )

    # @encode_msg
    # def
