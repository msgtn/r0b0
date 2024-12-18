"""
The base Gadget class
"""

# import logging
from r0b0 import logging
# logging = logging.getLogger(__name__)
from r0b0.config import LOCALHOST, SERVER_PORT, HEADER
from r0b0.utils.loaders import decode_msg, encode_msg

from socketio import Client, ClientNamespace

# from socketio import SimpleClient as Client, ClientNamespace
from threading import Thread
import urllib3

urllib3.disable_warnings()
import ssl

ssl._create_default_https_context = ssl._create_unverified_context

EVENTS = []


class Message(object):
    """Base class for representing a message

    A message encapsulates the packets that is sent through socket connections.
    Gadgets that require more processing to parse/produce messages should
    create their own message object that subclasses this
    """

    def __init__(self, *args, **kwargs):
        self.__dict__.update(**kwargs)

        # TODO - this is a kludge
        # Set solf dictionary to all information in the packet
        if kwargs.get("data", False):
            self.__dict__.update(**kwargs["data"])


class Gadget(Client, Thread):
    """Class representing a gadget

    More info...
    Even more info...


    :ivar name: The name of the gadget
    :ivar namespace: The namespace of the gadget for socket connections
    :ivar config: The configuration dictionary that defines the gadget
    :ivar hostname: The hostname that the gadget should connect to
    :ivar port: The port that the gadget should connect to
    :ivar message: The Message type
    """

    def __init__(self, config: dict = {"type": "Gadget", "name": "gadget"}, **kwargs):
        Client.__init__(
            self,
            ssl_verify=False,
            # logger=True,
            logger=False,
        )
        Thread.__init__(
            self,
            target=self._connect_thread,
            # daemon=True,
        )
        self._name = config.get("name", "")
        self.namespace = f'/{config.get("namespace",self.name)}'

        self.__dict__.update({"config": config})
        self.__dict__.update(**kwargs)
        self.config = config
        self.hostname = self.config.get("hostname", LOCALHOST)
        self.port = self.config.get("port", SERVER_PORT)
        # TODO - refactor to capital-M Message,
        # because this is a class; lowercase is for functions
        self.message = Message
        self.on(
            "call_method",
            handler=self.call_method_handler,
            namespace=self.namespace,
        )

    @decode_msg
    def call_method_handler(self, msg, *args, **kwargs):
        """
        Handler for calling a Gadget's method from an emit.

        :param msg: The message, should contain a "method" attribute.
        """
        if hasattr(self, msg.method):
            getattr(self, msg.method)(*args, **kwargs)
        else:
            logging.debug(f"Gadget {self.name} does not have method {msg.method}()")

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = value
        self.namespace = f"/{value}"

    def _connect_thread(
        self,
    ) -> None:
        url = f"{HEADER}://{self.hostname}:{self.port}{self.namespace}"
        logging.debug(f"{self.name} connecting to {url}")
        Client.connect(
            self,
            url=url,
            namespaces=[self.namespace, "/"],
            wait_timeout=20,
        )
        Client.wait(self)

    def _emit_ack(self):
        """Callback to check that the server"""
        pass

    def handle_events(self, EVENTS):
        for _event in EVENTS:
            self.on(
                _event,
                handler=getattr(self, f"{_event}_event"),
                namespace=self.namespace,
            )

    @encode_msg
    def emit(self, event, data, **kwargs) -> dict:
        """
        Emits a message.
        :param event: The event to emit.
        :return: The keyword arguments emitted.
        """
        # overwrite defaults
        data.update(dict(event=data.get("event", event), id=data.get("id", self.sid)))
        kwargs.update(
            dict(
                event=event,
                data=data,
            )
        )

        # TODO - kludge to avoid BadNamespaceError if trying to emit before the gadget is connected
        #
        try:
            Client.emit(self, **kwargs)
        except:
            pass
        return kwargs

    def assign_event_handlers(self, events_to_handle: list) -> None:
        """
        Assign handlers for events.

        :param events_to_handle: A list of strings of events to handle.
        """
        for event in events_to_handle:
            self.on(
                event, handler=getattr(self, f"{event}_event"), namespace=self.namespace
            )
            pass

    def handle_unassigned_event(self, data):
        """
        Handle an unassigned event by printing a debug message.

        :param data: The data attempted to be emitted.
        """
        logging.debug(f"{self.name} received unhandled event with data {data}")

    def disconnect(self) -> None:
        """
        Disconnect from the server if connected.
        """
        if self.connected:
            Client.disconnect(self)
        # if self.is_alive(): self.join()


def init_gadget(gadget_type=Gadget, *args, **kwargs) -> Gadget:
    return gadget_type(*args, **kwargs)


if __name__ == "__main__":
    gadget = init_gadget()
