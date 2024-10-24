from .gadget import Gadget, Message
import os, sys, logging, random

# logging = logging.getLogger(__name__)
# logging.basicConfig(
#     encoding='utf-8',
#     # level=logging.DEBUG,
#     level=logging.WARNING,
# )
import time
from time import sleep
from functools import partial
import threading
from r0b0.utils import loaders
from r0b0.utils.loaders import decode_msg, encode_msg

import serial

DEFAULT_CONFIG = {
    "type": "SerialGadget",
    "port": "8080",
    "serial_port": "/dev/cu.usbmodem1101",
    "timeout": None,
}

EVENTS = ["listen"]


class SerialGadget(Gadget):

    def __init__(self, config=DEFAULT_CONFIG, **kwargs):
        # breakpoint()
        Gadget.__init__(self, config, **kwargs)
        # serial.Serial.__init__(self, config['serial_port'], timeout=None)
        # self.serial = self._init_serial()
        self.POLL_SERIAL = True
        self.listen_thread = self._init_listen_thread()

        self.handle_events(EVENTS)
        self.listen()

    @decode_msg
    def listen_event(self, data):
        self.listen()

    def listen(self, start=True):
        # def listen(self):

        # self.listen_thread = self._init_listen_thread()
        # self.listen_thread.start()
        # time.sleep(10)
        # self.listen_thread.join()
        # return
        logging.warning("Serial gadget listening")

        if start:
            # if self.listen_thread is not None and self.listen_thread.is_alive():
            #     self.listen_thread.join()

            self.listen_thread = self._init_listen_thread()
            self.listen_thread.start()
            # time.sleep(10)
        # else:
        #     # self.POLL_SERIAL = False
        #     if self.listen_thread is not None and self.listen_thread.is_alive():
        #         self.listen_thread.join()

        # self.serial.close()
        # self.serial = self._init_serial()

    def _listen(self):
        # while self.serial.is_open():
        #     continue
        # self.serial = self._init_serial()
        # with self.serial as port:
        with self._init_serial() as port:
            # while True:
            # if self.POLL_SERIAL:
            # while port.in_waiting==0:
            #     continue
            res = port.read()

            # print(res)
            # with open('./tests/serial.txt','a') as _file:
            #     _file.write(res)
            #     _file.write('\n')
            self.emit(
                event="serial",
                data={"event": "serial", "value": True},
                # data={"event":"serial", "value":str(res)},
                namespace=self.namespace,
            )

            # else:
            #     return
        # self.listen()

    def _init_serial(self, **kwargs):
        return serial.Serial(
            port=self.config["serial_port"],
            timeout=self.config.get("timeout", DEFAULT_CONFIG["timeout"]),
            **kwargs
        )

    def _init_listen_thread(self):
        return threading.Thread(target=self._listen)
