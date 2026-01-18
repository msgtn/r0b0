from functools import partial
from collections import defaultdict
from multiprocessing import Process
import pickle
import time

import pygame
from pygame import (
    joystick as pgJoystick,
    event as pgEvent,
    display as pgDisplay,
    time as pgTime,
)

pygame.init()
pgDisplay.init()
pgJoystick.init()
pgTime.Clock().tick(1)
# try:
#     pgDisplay.init()
# except pygame.error:
#     pass


from r0b0 import gadgets as gadget_shelf, cables as r0b0_msgs, logging
from r0b0.gadgets.gadget import Message
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.utils import loaders
from r0b0.rigs.host import Host


class Rig(Host):
    """
    The Rig object wraps around the Host/server and connects Gadgets.
    """

    def __init__(self, hostname=LOCALHOST, port=SERVER_PORT, **kwargs):
        Host.__init__(self, hostname, port, namespaces="*", **kwargs)
        self.gadgets = {}
        self.hostname = hostname
        self.port = port
        self.power = False
        self.is_pygame_rig = False
        self.pygame_gadgets = {}
        # self.namespaces=namespaces

        # trying SIMO event handling
        self.event_handlers = defaultdict(list)
        self.__dict__.update(kwargs)

    def add_gadget(self, gadget_obj):
        #     config = loaders.load_gadget(gadget_name)
        #     gadget_cls = getattr(
        #         gadget_shelf, config['type'], None)
        #     assert gadget_cls is not None, f"Gadget type {config['type']} does not exist"
        #     gadget_obj = gadget_cls(config)

        # check if gadget class requires pygame
        if "pygame" in str(gadget_obj.__class__).lower():
            self.is_pygame_rig = True
            self.pygame_gadgets.update({gadget_obj.pygame_name: gadget_obj})

        # gadget_namespace = f"/{gadget_obj.name}"
        # if gadget_namespace not in self.namespaces:
        #     self.namespaces.append(gadget_namespace)
        #     self.server_options['namespaces'].append(gadget_namespace)

        # TODO - trying to handle multiple gadgets from the same config
        # check that gadget name does not already exist
        # if so, append index to name
        # gadget_name_idx = 1
        # _gadget_name_idx = gadget_name

        # while self.gadgets.get(_gadget_name_idx,None) is not None:
        #   _gadget+
        #   gadget_name_idx += 1

        self.gadgets.update({gadget_obj.name: gadget_obj})
        return self.gadgets

    def _get_gadget_namespace(self, gadget):
        # if gadget is None: return '/'
        if gadget is None:
            return None
        return self.gadgets.get(gadget).namespace

    def add_cable(self, cable, tx_gadget=None, rx_gadget=None):
        logging.info(f"Adding {cable} from {tx_gadget} to {rx_gadget}")
        assert not (
            tx_gadget is None and rx_gadget is None
        ), "Either or both tx_gadget and rx_gadget must be defined when calling add_cable()"
        if tx_gadget is None:
            tx_gadget = rx_gadget
        if tx_gadget.name not in self.gadgets:
            self.add_gadget(tx_gadget)
        if rx_gadget is not None:
            if rx_gadget.name not in self.gadgets:
                self.add_gadget(rx_gadget)
        else:
            rx_gadget = tx_gadget
        tx_namespace, rx_namespace = tx_gadget.namespace, rx_gadget.namespace
        input_event = cable.input_event

        def func_emit(data):
            # if not isinstance(data,dict): data = pickle.loads(data)
            # msg_kwargs = msg_func(data)
            # Handle SEQUENTIAL single-input multiple-output
            msg_kwargs_list = cable(data)
            if not isinstance(msg_kwargs_list, list):
                msg_kwargs_list = [msg_kwargs_list]
            for msg_kwargs in msg_kwargs_list:
                logging.debug(msg_kwargs)
                if msg_kwargs is None:
                    return
                # if "namespace" in msg_kwargs:
                #     _namespace = msg
                # _namespace
                # wrap the data into the gadget's expected message object
                if rx_gadget is None or rx_gadget == tx_gadget:
                    emit_data = Message(**msg_kwargs)
                    include_self = True
                else:
                    emit_data = self.gadgets[rx_gadget.name].message(**msg_kwargs)
                    include_self = False
                output_event = emit_data.event
                # assemble the output to emit
                emit_kwargs = dict(
                    event=output_event,
                    data={"event": output_event, "msg": pickle.dumps(emit_data)},
                    to=None,
                    include_self=include_self,
                    # namespace=rx_namespace,
                    namespace=msg_kwargs.get("namespace", rx_namespace),
                )
                # breakpoint()
                # logging.debug(f"func_emit {emit_kwargs}")
                # print('func_emit', emit_kwargs)
                self.emit(**emit_kwargs)
                # time.sleep(2)

        input_handlers = self.event_handlers.get(input_event, [])
        input_handlers.append(func_emit)
        self.event_handlers.update({input_event: input_handlers})
        #
        self.on_event(
            input_event,
            # handler=func_emit,
            handler=partial(self.multi_handler, input_event=input_event),
            namespace=tx_namespace,
        )
        pass

    def add_cable_func(self, cable, tx_gadget=None, rx_gadget=None):
        # logging.debug('add_message',tx_gadget, rx_gadget, msg_func)

        tx_namespace, rx_namespace = map(
            self._get_gadget_namespace, [tx_gadget, rx_gadget]
        )
        msg_func = getattr(r0b0_msgs, cable)
        input_event = msg_func()["event"]

        # logging.debug( tx_namespace, rx_namespace, input_event )
        def func_emit(data):
            # if not isinstance(data,dict): data = pickle.loads(data)
            msg_kwargs = msg_func(data)
            if msg_kwargs is None:
                return
            # wrap the data into the gadget's expected message object
            if rx_gadget is None:
                emit_data = Message(**msg_kwargs)
                include_self = True
            else:
                emit_data = self.gadgets[rx_gadget].message(**msg_kwargs)
                include_self = False
            output_event = emit_data.event
            # assemble the output to emit
            emit_kwargs = dict(
                event=output_event,
                data={"event": output_event, "msg": pickle.dumps(emit_data)},
                to=None,
                include_self=include_self,
                namespace=rx_namespace,
            )
            logging.debug(f"func_emit {emit_kwargs}")
            # print(emit_kwargs)
            self.emit(**emit_kwargs)

        input_handlers = self.event_handlers.get(input_event, [])
        input_handlers.append(func_emit)
        self.event_handlers.update({input_event: input_handlers})
        #
        self.on_event(
            input_event,
            # handler=func_emit,
            handler=partial(self.multi_handler, input_event=input_event),
            namespace=tx_namespace,
        )

    def multi_handler(self, data, input_event):
        """
        Wrapper to handle single-input multiple-output callbacks
        """
        for handler_func in self.event_handlers.get(input_event, []):
            handler_func(data)

    def pygame_event_handler(self):
        """
        Loop to handle a rig with PyGame objects
        Infinite loop to catch PyGame events
        """
        T_LAST_EVENT = time.time()
        T_COOLDOWN = 300 / (10e3)
        logging.debug("GOING INTO PYGAME EVENT HANDLER")
        while True:
            if (time.time() - T_LAST_EVENT) < T_COOLDOWN:
                continue
            T_LAST_EVENT = time.time()
            for event in pgEvent.get():
                logging.debug(event)
                _event_name = pgEvent.event_name(event.type).lower()
                _event_dict = event.__dict__
                pygame_name = ""

                if "joy" in _event_name:
                    pygame_name = f'joy_{_event_dict.get("joy","")}'

                elif "key" in _event_name:
                    pygame_name = "keys"

                logging.debug(pygame_name)
                event_gadget = self.pygame_gadgets.get(pygame_name, None)
                emit_dict = dict(
                    event=_event_name,
                    data=_event_dict,
                )
                logging.debug(event_gadget)
                # logging.debug(_event_name)
                # logging.debug(pygame_name)
                if event_gadget is not None:
                    # logging.debug(event_gadget.namespace)
                    emit_dict.update(dict(namespace=event_gadget.namespace))
                    # logging.debug(emit_dict)
                    # logging.debug(emit_dict)
                    event_gadget.emit(**emit_dict)
                break

    def power_on(
        self,
    ):
        logging.info("POWER ON")

        self.start()
        logging.debug(self.gadgets.values())
        [g.start() for g in self.gadgets.values()]
        self.power = True
        # self.emit(event='set_mode',data={
        #         'event':'set_mode',
        #         'mode':'stopwatch',
        #         'namespace':'/time_controller'
        #         },
        #         # include_self=False,
        #         namespace='/time_controller'
        #     )
        if self.is_pygame_rig:
            self.pygame_event_handler()

    def power_off(self, *args, **kwargs):

        assert self.power or self.is_alive(), "Rig not powered on"
        # self.disconnect()
        self.join()
        [g.disconnect() for g in self.gadgets.values()]
        self.power = False
