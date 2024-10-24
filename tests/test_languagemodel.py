import pytest

import pytest
import uuid
import os, time, pickle, json
import requests

# print(uuid.uuid4())
import r0b0
from r0b0.gadgets import Gadget
from r0b0.gadgets.dxl_robot import *
from r0b0.gadgets.language_model import LanguageModel
from r0b0.gadgets.time_controller import *
from flask_socketio import SocketIO, Namespace
from flask import jsonify
import socketio
from timeit import default_timer

@pytest.fixture
def lm():
    return LanguageModel()

class TestLanguageModel:


    def test_init(self, lm):

        res = lm.prompt("testing")
        breakpoint()

    # @pytest.mark.parametrize(
    #     "text",
    #     ["hello world",
    #     "the medium is the message",
    #     "marx was right",
    #     "the mitochondria is the powerhouse of the cell"]
    # )
    # def test_voice(self, lm, text):
    #     lm.voice(text)
    #     breakpoint()