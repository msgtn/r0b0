import sched
from r0b0.config import CSR_PEM, KEY_PEM, LOCALHOST, SERVER_PORT, TAPES_DIR
from .gadget import Gadget, Message

import os
import json
from collections import OrderedDict
from socketio import ClientNamespace
from threading import Thread
from flask import render_template
import time
from socketio import Client
import pickle
import logging

logging.basicConfig(
    filename="phone.log",
    encoding="utf-8",
    level=logging.DEBUG,
)


class Tape(Gadget):
    """
    A tape of recorded socket events
    """

    def __init__(self, name="tmp", hostname=LOCALHOST, port=SERVER_PORT, **kwargs):
        Gadget.__init__(self, config={"name": name}, hostname=hostname, port=port)
        self.filename = str(TAPES_DIR / f"{name}.json")
        self.tape = []
        self.sched = sched.scheduler(
            timefunc=time.time,
            delayfunc=time.sleep,
        )
        self.playing_thread = Thread(
            target=self._play,
        )
        self.playing = False
        self.started = False

    @classmethod
    def load(self, name):
        filename = str(TAPES_DIR / f"{name}")
        if not filename[-5:] == ".json":
            filename = f"{filename}.json"
        # assert os.path.exists(filename), "No"
        if os.path.exists(filename):
            tape = Tape(name)
            tape.tape = tape.open(filename)
            tape._normalize_time()
            return tape
        else:
            logging.warning(
                f"Tape '{name}' could not be loaded, does it exist in ./tapes/ ?"
            )
            return None

    def start(self):
        self.namespace = self.tape[0].get("namespace", "")
        logging.debug(f"Tape namespace: {self.namespace}")
        # connect to server
        Gadget.start(self)
        # while len(self.namespaces)==0:
        max_retries = 100
        while max_retries > 0 and self.namespaces.get(self.namespace, None) is None:
            # continue
            logging.debug("Waiting for client connect")
            max_retries -= 1

        logging.debug(self.namespaces)
        self.started = True

    def play(self, **kwargs):
        if not self.started:
            self.start()
        self.playing = True
        logging.debug(kwargs)
        self.playing_thread = Thread(
            target=self._play,
            kwargs=kwargs
        )
        self.playing_thread.start()

    def _play(self, loop=False):
        while True:
            t_last = 0
            for f, frame in enumerate(self.get_frame()):
                if not self.playing:
                    break
                # logging.debug(frame)
                self.emit(**frame)
                # print(frame)
                time_sleep = frame["data"]["time"] - t_last
                time.sleep(time_sleep)
                t_last = frame["data"]["time"]
            if not loop or not self.playing:
                break
            else:
                time.sleep(time_sleep)
                logging.debug("looping")

        self.playing = False
        logging.debug("Done playing tape")

    def stop(self,):
        self.playing = False

    def _normalize_time(self):
        t_0 = self.tape[0]["data"]["time"]
        for f, frame in enumerate(self.tape):
            self.tape[f]["data"]["time"] = (frame["data"]["time"] - t_0) / 10e3

    def open(self, tape_name, open_mode="r"):
        with open(tape_name, open_mode) as tape_file:
            return json.loads("".join(tape_file.readlines()))

    def write(self, frame):
        # TODO - check that the frame is valid, e.g. has event name and time is after last time
        self.tape.append(frame)

    def save(self):
        assert self.name is not "tmp", "Name for tape not defined, can't save"
        assert len(self.tape) > 0, f"Empty tape {self.name}, not saving"
        with open(self.filename, "w") as tape_file:
            tape_file.write(json.dumps(self.tape, indent=4))

    def get_frame(self, in_idx=0, out_idx=-1):
        # TODO - check if hit end of tape
        assert abs(in_idx) < len(self.tape) and abs(out_idx) < len(
            self.tape
        ), "Indices longer than tape"
        subtape = self.tape[in_idx:out_idx]
        for f, frame in enumerate(subtape):
            # print(frame['data']['time'])
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
