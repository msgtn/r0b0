from .gadget import Gadget, Message
import logging
from r0b0.utils import loaders
from r0b0.utils.loaders import decode_msg, encode_msg
import llm

DEFAULT_CONFIG = {
    "type": "LanguageModel", 
    "model": "llama3"
}
EVENTS = ["prompt"]

class LanguageModel(Gadget):
    def __init__(self, config=DEFAULT_CONFIG, **kwargs):
        Gadget.__init__(self , config, **kwargs)
        self.model = llm.get_model(config["model"])
        self.handle_events(EVENTS)

    def prompt(self, prompt_string):
        return self.model.prompt(prompt_string).text()

    @decode_msg
    def prompt_event(self, data):
        msg = data["msg"]
        # breakpoint()
        print(msg)
        print(self.prompt(msg.prompt_string))
