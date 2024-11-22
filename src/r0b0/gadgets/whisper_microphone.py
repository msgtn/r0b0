from r0b0.gadgets.microphone import *
from r0b0.gadgets import Gadget

import whisper
import librosa

class WhisperMicrophone(Microphone):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.model = whisper.load_model("base.en")