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
    # "timeout": None,
    "timeout": 2,
}

EVENTS = ["listen", "write"]


class SerialGadget(Gadget):

    def __init__(self, config=DEFAULT_CONFIG, **kwargs):
        # breakpoint()
        Gadget.__init__(self, config, **kwargs)
        # serial.Serial.__init__(self, config['serial_port'], timeout=None)
        # self.serial = self._init_serial()
        self.POLL_SERIAL = True
        self.listen_thread = self._init_listen_thread()

        self.handle_events(EVENTS)
        self.listening = False
        # self.listen()

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
            # self.listening = True
            # time.sleep(10)
        # else:
        #     # self.POLL_SERIAL = False
        #     if self.listen_thread is not None and self.listen_thread.is_alive():
        #         self.listen_thread.join()

        # self.serial.close()
        # self.serial = self._init_serial()

    def _listen(self):
        if self.listening:
            return
        # while self.serial.is_open():
        #     continue
        # self.serial = self._init_serial()
        # with self.serial as port:
        detected = False
        # detected_time = 0
        with self._init_serial() as port:
            self.listening = True
            # while True:
            # if self.POLL_SERIAL:
            # while port.in_waiting==0:
            #     continue
            while True:
                if self.listening==False:
                    break
                try:
                    res = port.readline()
                except:
                    continue
                # print(res)
                res = res.decode().strip(' \t\n\r')
                # print(res)
                detected = res=="detected"
                # detected = "detected" in res
                # print(res, detected)
                # with open('./tests/serial.txt','a') as _file:
                #     _file.write(res)
                #     _file.write('\n')
                if detected:
            # if detected:
                    self.emit(
                        event="serial",
                        data={"event": "serial", "detected": detected},
                # data={"event":"serial", "value":str(res)},
                        namespace=self.namespace,
                    )
                    break
                time.sleep(0.1)
            self.listening = False
        logging.warning("Stopped listening")
            # else:
            #     return
        # if not detected:
        #     self.listen()

    def _init_serial(self, **kwargs):
        # breakpoint()
        # baudrate = kwargs.get("baudrate")
        if "baudrate" not in kwargs:
            kwargs["baudrate"] = 10_000_000
        return serial.Serial(
            port=self.config["serial_port"],
            timeout=self.config.get("timeout", DEFAULT_CONFIG["timeout"]),
            **kwargs
        )

    def _init_listen_thread(self):
        return threading.Thread(target=self._listen)

    def _write(self, text, **kwargs):
        with self._init_serial(**kwargs) as port:
            # port.writelines(bytes(text,'utf-8'))
            port.write(f"{text}".encode('utf-8'))
            # reply = port.read_until('\r'.encode('utf-8'))
            # breakpoint()
    
    @decode_msg
    def write_event(self, data):
        # msg = data.msg
        txt = data['msg'].text.replace("'","")
        self.write(txt)
    
    def write(self, txt):
        # if msg in 
        self._write("\x03\r\f", baudrate=10_000_000, write_timeout=2)
        time.sleep(1.5)
        txt = 'print_text("' + txt + '")\r\f'
        # self._write(f"print_text('{txt}')\r\f", baudrate=10_000_000, write_timeout=2)
        self._write(txt, baudrate=10_000_000, write_timeout=2)
        time.sleep(3)
        self._write(f"main()\r\f", baudrate=10_000_000, write_timeout=2)
        # breakpoint()