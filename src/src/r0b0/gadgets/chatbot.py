from .gadget import Gadget, Message
from r0b0.utils.loaders import decode_msg
import logging

from simpleaichat import AIChat
from getpass import getpass

EVENTS = ["prompt"]


class ChatBot(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        # AIChat.__init__(self,getpass('OpenAI key: '))
        self.bot = AIChat(api_key=getpass("OpenAI key: "), console=False)
        self.handle_events(EVENTS)

    @decode_msg
    def prompt_event(self, data):
        msg = data["msg"]
        logging.warning(data)
        res = self.bot(msg.prompt)

        self.emit(
            event="response",
            data={"event": "response", "response": res},
            namespace=self.namespace,
        )
        logging.warning(res)
