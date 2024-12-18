import glob, inspect

# import logging

# logging = logging.getLogger(__name__)
import datetime

from r0b0.config import (
    ROOT_DIR,
    TAPES_DIR,
    GADGETS_DIR,
    STATIC_DIR,
    PUBLIC_DIR,
    LOCALHOST,
    SERVER_PORT,
    CSR_PEM,
    KEY_PEM,
    BROWSER_DIR,
    SOCKET_ADDR,
    HEADER,
)
from r0b0.utils.loaders import decode_msg, encode_msg
from r0b0.gadgets import Tape
from r0b0 import logging, get_timestamp

import os
from aiohttp import web
import socketio
from socketio import AsyncServer, Server, Namespace
import pickle
from functools import partial, partialmethod, wraps
from threading import Thread
from copy import copy, deepcopy

import sched
from collections import OrderedDict

from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import requests

# import eventlet
# eventlet.monkey_patch()
# def listen():
#     while True:
#         # bg_emit()
#         eventlet.sleep(5)

# eventlet.spawn(listen)
import time
import json

# TODO - try to remove dependency of having to import Thread first
# to call the correct *.run()


PLAYER_EVENTS = [
    "load",
    "play",
    "stop",
    "record",
    "echo"
]


class Host(Thread, SocketIO):
    """
    The Host object serves socket connections.
    Host subclasses Thread and SocketIO.
    """

    def __init__(
        self,
        hostname=LOCALHOST,
        port=SERVER_PORT,
        # certfile=CSR_PEM, keyfile=KEY_PEM,
        certfile=None,
        keyfile=None,
        pages_folder=None,
        socket_addr=SOCKET_ADDR,
        **kwargs,
    ):
        flask_kwargs = {}
        if pages_folder:
            flask_kwargs.update(
                {
                    # 'template_folder':os.path.join(pages_folder,'templates'),
                    # 'static_folder':os.path.join(pages_folder, 'static'),
                    # 'static_url_path':os.path.join(pages_folder, 'static'),
                    "root_path": pages_folder,
                }
            )

        # Write the socket address to the static folder so it can be imported by the page
        if pages_folder:
            with open(
                os.path.join(pages_folder, "static", "socket_addr.js"), "w"
            ) as _file:
                print(f"Writing {socket_addr} to {_file}")
                _file.write(f'let socketAddr = "{socket_addr}";')

        self.app = app = Flask(__name__, **flask_kwargs)
        CORS(self.app)
        self.hostname = hostname
        self.port = port
        # print(f'host port {port}')
        # print(bytes(SOCKET_ADDR,'utf-8'), bytes(socket_addr,'utf-8'))
        # print(type(SOCKET_ADDR), type(socket_addr))
        SocketIO.__init__(
            self,
            self.app,
            cors_allowed_origins=[
                "*",
                socket_addr,
                f"{HEADER}://{self.hostname}:{self.port}",
                # f"{HEADER}://{self.hostname}:{self.port}",
            ],
            max_http_buffer_size=1e8,
            # async_mode='threading',
            # async_mode='eventlet',
            **kwargs,
        )
        # eventlet.monkey_patch()
        # self.app.wsgi_app = socketio.WSGIApp(self, self.app.wsgi_app)
        self.socketio_run = partial(SocketIO.run, self)
        self.thread_kwargs = {
            "host": self.hostname,
            "port": self.port,
            "certfile": certfile,
            "keyfile": keyfile,
        }

        Thread.__init__(
            self,
            # TODO - as above, in order for this to work, must subclass Thread before SocketIO
            # because they both have run() functions
            # target = SocketIO.run,
            target=self.start_wrapper,
            args=(self, self.app),
            kwargs=self.thread_kwargs,
        )

        # Special handlers for Page gadgets
        SocketIO.on_event(
            self,
            "add_url",
            self.add_url,
        )
        SocketIO.on_event(
            self,
            "add_emit",
            self.add_emit,
        )

        SocketIO.on_event(
        # self.server.on(
            self,
            "stopControl",
            lambda : partial(self.manual_emit, event="stopControl", data={"event":"stopControl"})
        )

        self._webrtc_setup()
        self._player_setup()

        # self.power_on, self.power_off = self.start, self.join

    @encode_msg
    def manual_emit(self, event, data, *args, **kwargs):
        """Manually emit an event.
        This is a helper function to emit messages without cables,
        such as in a script or in the CLI.
        This works around well-documented problems of using
        threads with sockets.

        :param event: The event to emit
        """
        assert data is not None, f"Rig cannot emit event {event} without data"
        data.update(
            {
                "args": args,
                "kwargs": kwargs,
            }
        )

        # Post a request to the Flask server
        res = requests.post(
            f"{HEADER}://{self.hostname}:{self.port}/forward",
            data=json.dumps(data),
            headers={"Content-Type": "application/json"},
            verify=False,
        )

    def _forward_route(
        self,
    ):
        """Forwarding route to emit an event manually.
        This is a helper function that should be posted to by
        Host.manual_emit().

        :return: Server response
        """

        data = json.loads(request.data)
        data_args = data["args"]
        data_kwargs = data["kwargs"]

        event = data["event"]
        data_kwargs = {
            "data": data,
            **data["kwargs"],
        }
        self.emit(event, *data_args, **data_kwargs)
        return "emitted", 200

    def start_wrapper(self, *args, **kwargs):
        self.app.add_url_rule(
            "/forward", view_func=self._forward_route, methods=["POST"]
        )
        SocketIO.run(*args, **kwargs)

    # Not using the encode_msg decorator here because
    # the message should have been encoded in an earlier emit function.
    def emit(self, event, *args, **kwargs):
        """Emit an event.
        Positional and keyword arguments can contain data,
        namespaces, and whatever else socket.emit() uses.
        This function is often just a coupling between two gadgets.

        :param event: The event
        """
        logging.debug(f"HOST EMIT {event} {datetime.datetime.now()}")
        # logging.debug(args)
        # logging.debug(kwargs)

        # if the event is a player-related event,
        # handle it internally
        if event in PLAYER_EVENTS:
            getattr(self, f"on_{event}")(*args, **kwargs)
        else:
            if kwargs.get("namespace", None)=="/tape":
                kwargs.update({"namespace":kwargs["rx_namespace"]})
            logging.debug(f"{args}, {kwargs}")
            SocketIO.emit(self, event, *args, **kwargs)

    @decode_msg
    def add_url(self, data):
        """Route to a URL.
        Used only for the Page gadget.

        :param data: The data packet
        """
        route_func = lambda: render_template(data["url"])
        route_func.__name__ = f"route_{data['url'].split('.')[0]}"
        self.app.add_url_rule(data["route"], view_func=route_func)

    @decode_msg
    def add_emit(self, data):
        """Add an emit function.
        Used only for the Page gadget.

        :param data: The data packet
        """
        logging.debug("add_emit")
        logging.debug(data)
        event = data["event"]

        def _emit_record(s, d):
            """Wrapper function that enables recording emitted event

            :param s: _description_
            :param d: _description_
            """
            logging.debug(s)
            logging.debug(d)

            self.emit(
                event=d["event"],
                data=d,
                **data["kwargs"],  # namespace arg from the .yaml that defined it
            )
            if d.get("id", None) is not None:
                id_event = f"{d['id']}_{d['event']}"
                tape = self.tapes.get(id_event, None)
                if tape is not None:
                    # if id_event in self.tapes.keys():
                    # record time in millis
                    d.update(
                        {
                            "time": int(time.time() * 10e3),
                        }
                    )
                    tape.write(
                        {
                            **{"event": data["event"], "data": d},
                            **data["kwargs"],
                        }
                    )

        self.server.on(
            data["event"],
            _emit_record,
            # TODO - figure out how to have this on device-specific namespace
            namespace="/",
            # **data['kwargs']
        )

    # metaphor - VCR player
    def _player_setup(self):
        """Set up the tape player"""
        self.tapes = OrderedDict()
        for player_event in PLAYER_EVENTS:
            SocketIO.on_event(self, player_event, getattr(self, f"on_{player_event}"))
        self.app.add_url_rule(
            "/tapes",
            view_func=self.get_tapes,
        )

    def get_tapes(self):
        """Return a list of available tapes

        :return: A list of available tapes
        """
        # tapes = send_from_directory()
        return sorted(os.listdir(TAPES_DIR))

    def on_load(self, data):
        logging.debug(data)
        tape_name = data["tape_name"]
        if tape_name in self.tapes.keys():
            return self.tapes[tape_name]
        tape = Tape.load(tape_name)
        if tape:
            self.tapes.update({tape_name: tape})
        return tape

    def on_record(self, data):
        """
        data = {
            record: true(start)/false(stop),
            event: str,
        }
        """
        # Create an internal identifier of {id}_{event}
        id_event = f"{data['id']}_{data['event']}"

        # data['record'] is boolean
        if data["record"]:
            # start recording, make a new Tape
            tape_name = f"{get_timestamp()}_{data['event']}"
            self.tapes.update({id_event: Tape(tape_name)})
        else:
            # stop recording, get the Tape and save
            tape = self.tapes.pop(id_event, None)
            if tape:
                tape.save()
        logging.debug(self.tapes)

    @decode_msg
    def on_play(self, data, **kwargs):
        if "msg" in data:
            data.update(data["msg"].__dict__)
        tape = self.tapes.get(data["tape_name"], None)
        loop = self.tapes.get(data["loop"], False)
        loop = data.get('loop', False)
        # breakpoint()
        if tape is None and "msg" in data:
            # tape = self.tapes.getattr(data['msg'],'tape_name',None)
            tape = self.tapes.get(getattr(data["msg"], "tape_name", None), None)
            loop = getattr(data["msg"], "loop", False)
        logging.debug(f"tape {tape}, loop: {loop}")

        if tape is not None:
            tape.play(loop=loop)
        else:
            # try to load tape
            tape = self.on_load(data)
            if tape is not None:
                tape.play(loop=loop)
            # could not load tape
            else:
                logging.warning(f"No tape {data['tape_name']}, cannot play")

    # TODO - wrapper play function for CLI usage
    def play(self, tape_name):
        """Play a tape

        :param tape_name: The name of the tape to play
        """
        self.on_load({"tape_name": tape_name})
        self.on_play({"tape_name": tape_name})

    def stop(self, tape_name):
        self.on_stop({"tape_name": tape_name})

    @decode_msg
    def on_stop(self, data, **kwargs):
        if "msg" in data:
            data.update(data["msg"].__dict__)
        tape_name = data.get("tape_name", None)
        if tape_name is None:
            logging.debug("Trying to stop all tapes")
            for tape in self.tapes.values():
                if tape.playing:
                    logging.debug(f"Stopping {tape}")
                    tape.stop()
        else:
            tape = self.tapes.get(data["tape_name"], None)
            if tape is None and "msg" in data:
                # tape = self.tapes.getattr(data['msg'],'tape_name',None)
                tape = self.tapes.get(getattr(data["msg"], "tape_name", None), None)
            logging.debug(f"tape {tape}")

            if tape is not None:
                tape.stop()

    @decode_msg
    def on_echo(self, data, **kwargs):
        logging.debug("echoing")
        kwargs.update({
            "namespace":data["rx_namespace"],
            # "event":data["event"]
        })
        # breakpoint()
        self.emit(event=data["event"], data=data, **kwargs)

    # no metaphor for this one
    def _webrtc_setup(self):
        self.broadcaster_id = None
        webrtc_events = ["broadcaster", "watcher", "offer", "answer", "candidate"]
        for webrtc_event in webrtc_events:
            SocketIO.on_event(self, webrtc_event, getattr(self, webrtc_event))

    def broadcaster(self, sid):
        self.broadcaster_id = sid
        SocketIO.emit(self, "broadcaster")

    def watcher(self, sid):
        if not self.broadcaster_id:
            return
        SocketIO.emit(
            self,
            "watcher",
            request.sid,
            to=self.broadcaster_id,
        )

    def offer(self, sid, msg, *args, **kwargs):
        SocketIO.emit(
            self,
            "offer",
            (request.sid, msg),
            to=sid,
        )

    def answer(self, sid, msg):
        # TODO - handle max connections
        SocketIO.emit(
            self,
            "answer",
            (request.sid, msg),
            to=sid,
        )

    def candidate(self, sid, msg):
        SocketIO.emit(
            self,
            "candidate",
            (request.sid, msg),
            to=sid,
        )


if __name__ == "__main__":
    host = Host()
    host.start()
    breakpoint()
