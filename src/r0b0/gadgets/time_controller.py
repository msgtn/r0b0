from enum import Enum
import time

# from .gadget import Gadget, Message, logging
from .gadget import Gadget, Message
import logging

# logger = logging.getLogger(__name__)
logger = logging.getLogger()
from r0b0.utils.loaders import decode_msg
from timeit import default_timer
from threading import Thread
import multiprocessing
from functools import partial


# MODES = ['idle','stopwatch','timer']
class TimeMode(Enum):
    IDLE = idle = 1
    STOPWATCH = stopwatch = 2
    TIMER = timer = 3


EVENTS = [
    "set_mode",
    # "set_position"
]


class TimeController(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(
            self,
            config,
            # request_timeout=0.1,
            # *args,
            **kwargs,
        )
        self.handle_events(EVENTS)
        self.mode = TimeMode.IDLE
        self.ticking = False
        self.tick_thread = None
        self.position = 0
        self.position_buffer = []
        self.rotations = 0
        self.timer = None
        self.last_moved_time = self.start_time = 0

    @decode_msg
    def set_position_event(self, data):
        self.position = data["msg"].position

    @staticmethod
    def _position2timer(position, rotations=0, ccw_positive=True):
        # return (4096-position)
        ret = position
        if ccw_positive:
            ret = 4096 - position
        ret *= 60 / 4096
        ret += rotations * 60
        return ret

    @staticmethod
    def _timer2position(timer):
        position = 0
        return int(4096 - (timer % 60) * 4096 / 60)

    @decode_msg
    def set_mode_event(self, data):
        msg = data["msg"]
        last_mode = self.mode
        self.last_moved_time = default_timer()
        self.mode = getattr(TimeMode, msg.mode.upper())

        logging.debug(f"SETTING MODE {self.mode}, {self.ticking}")
        self.position = msg.position
        # if len(self.position_buffer):
        #     if self.position_buffer[-1] < self.position:
        #         self.rotations  += 1

        self.position_buffer.append(self.position)

        # logging.debug(f"POSITION, {self.rotations}, {self.position_buffer}")
        # "debounce"
        if self.ticking:
            logging.debug("TICKING")
            self.ticking = False
            self.mode = TimeMode.IDLE
            if self.tick_thread is not None:
                logging.debug("KILLING TICK THREAD")
                self.tick_thread.join()
                # self.tick_thread.terminate()
                self.tick_thread = None
            logging.debug("Resetting to IDLE")
        elif last_mode != self.mode:
            if self.tick_thread is not None:
                if self.tick_thread.is_alive():
                    return
            logging.debug(self.mode)
            if self.mode == TimeMode.STOPWATCH:
                self.direction = -1
            elif self.mode == TimeMode.TIMER:
                self.direction = 1
            elif self.mode == TimeMode.IDLE:
                self.direction = 0
                self.position_buffer = []
                self.rotations = 0
            if self.tick_thread is not None:
                self.tick_thread.join()
                # self.tick_thread.terminate()
                self.tick_thread = None
                self.ticking = False
            if self.mode != TimeMode.IDLE:
                # self.tick_thread = multiprocessing.Process(
                #     target=partial(self._tick_thread, direction=self.direction)
                # )
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

    def _tick_thread(self, direction=1, tick_period=1, *args, **kwargs):
        # "Debounce" / timeout, only start thread after 2 seconds of no motion
        while default_timer() - self.last_moved_time < 2:
            continue
        self.ticking = True

        logging.debug(f"Starting tick thread with {direction:}")
        self.timer = TimeController._position2timer(self.position, self.rotations)
        logging.debug(f"TIMER, {self.timer}, {self.position}")
        last_time = self.start_time = default_timer()
        while self.timer > 0 and self.ticking:
            if default_timer() - last_time >= tick_period:
                self._tick(direction, *args, **kwargs)
                last_time = default_timer()
        self.mode = TimeMode.IDLE
        self.ticking = False
        self.emit(
            event="idle",
            data={
                "event": "idle",
            },
            namespace=self.namespace,
        )
        time.sleep(2)
        # self.tick_thread.join()

    def _tick(self, direction=1):
        logging.debug(f"tick {direction}")
        # self.position += direction*(4096//60)
        # self.position %= 2**12
        # if direction == 1:
        self.timer -= direction * (default_timer() - self.last_moved_time)
        if self.timer <= 0:
            return
        self.last_moved_time = default_timer()
        self.position = TimeController._timer2position(self.timer)
        logging.debug(f"TIMER, POSITION {self.timer}, {self.position}")
        self.emit(
            event="tick",
            data={
                "event": "tick",
                "direction": direction,
                "position": self.position,
                "msg": Message(event="tick", direction=direction),
            },
            namespace=self.namespace,
        )

    # @encode_msg
    # def
