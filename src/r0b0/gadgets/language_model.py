from .gadget import Gadget, Message
import logging
from r0b0.utils import loaders
from r0b0.utils.loaders import decode_msg, encode_msg
import llm

class LanguageModel(Gadget):
    def __init__(self):
        Gadget.__init__(self , config, **kwargs)
        