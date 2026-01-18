from r0b0.config import CSR_PEM, KEY_PEM
from .gadget import Gadget, Message
import logging
from r0b0.utils.loaders import decode_msg

from collections import OrderedDict
from socketio import Client, ClientNamespace
import numpy as np
from threading import Thread
from flask import render_template
import time

import pickle


class Page(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.route_urls = config.get("route_urls", {})
        self.event_kwargs = config.get("event_kwargs", {})
        self.on(
            "*",
            handler=self.on_catch_all,
            namespace=self.namespace,
        )

    @decode_msg
    def on_catch_all(self, data):
        """Generic handler for events that do not have defined handler functions

        :param data: The data packet
        """
        if isinstance(data, str):
            event = data
            data = {"event": event}
        else:
            event = data.get("event", "unknown_event")
        logging.debug(f"Page {self.name} received {event}")
        print(f"Page {self.name} received {event}")

        self.emit(
            event=event,
            data=data,
            namespace=self.namespace,
        )

    # def add_emit(self,event,**kwargs):
    #     self.on(
    #         event,
    #         **kwargs,
    #     )

    @decode_msg
    def on_record(self, data):
        logging.debug(data)

    def start(self):
        Thread.start(self)

        # sleep to wait until connection is established
        logging.debug("Connecting Page...")
        time.sleep(2)

        # route urls
        for _route, _url in self.route_urls.items():
            self.emit(event="add_url", data={"route": _route, "url": _url})

        # Define event handlers
        # These appear as a nested dictionary in the config.yaml
        # event_kwargs = {
        #   event_1: {
        #       event_1_kwarg_1_key: event_1_kwarg_2_value,
        #       event_1_kwarg_2_key: event_1_kwarg_2_value
        # }}
        for _event, _kwargs in self.event_kwargs.items():
            if _kwargs is None:
                _kwargs = {}
            _kwargs.update(dict(namespace=self.namespace))
            logging.debug(f"{_event},{_kwargs}")
            self.emit(
                event="add_emit",
                data={"event": _event, "kwargs": _kwargs},
            )
            # The event kwarg should match the function name,
            # e.g. event_1 will call this object's 'on_event_1' function
            # If the function is not defined, the event will be handled
            # by the default 'on_catch_all' event
            self.on(
                _event,
                handler=getattr(self, f"on_{_event}", self.on_catch_all),
                # lambda msg: print(f"Page received {_event} event")),
                namespace=self.namespace,
            )

    @decode_msg
    def on_file_upload(self, data):
        # print('device_motion', data, self.namespace)
        self.emit(
            event="file_upload",
            data=data,
            namespace=self.namespace,
        )


class MobilePage(Page):
    def __init__(self, *args, **kwargs):
        Page.__init__(self, *args, **kwargs)

    @decode_msg
    def on_device_motion(self, data):
        # print('device_motion', data, self.namespace)
        self.emit(
            event="device_motion",
            data=data,
            namespace=self.namespace,
        )

    @decode_msg
    def on_text(self, data):
        # print('device_motion', data, self.namespace)
        self.emit(
            event="text",
            data=data,
            namespace=self.namespace,
        )

    @decode_msg
    def on_stopControl(self, data):
        logging.debug(f"Stop Control {data}")

        self.emit(
            event="stopControl",
            data={"event":"stopControl"},
            # include_self=True,
            # broadcast=True,
            # include_self=False,
            # namespace=self.namespace,
            # namespace="/"
        )