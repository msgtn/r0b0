from .gadget import Gadget, Message
from r0b0.utils.loaders import decode_msg
import logging

import speech_recognition as sr

EVENTS = [
    'listen'
]

# class Camera(Gadget):
class Microphone(Gadget, ):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self,config,**kwargs)
        self.rec = sr.Recognizer()
        self.mic = sr.Microphone()
        with self.mic as source:
            logging.warning('Calibrating microphone for ambience')
            self.rec.adjust_for_ambient_noise(source)
        self.handle_events(EVENTS)
    
    @decode_msg        
    def listen_event(self,data):
        msg = data['msg']
        logging.warning(f'{self.name} now listening')
        with self.mic as source:
            text = self.rec.recognize_vosk(
                self.rec.listen(source))
            self.emit(
                event='text',
                data={
                    'event':'text',
                    'text':text
                },
                namespace=self.namespace
            )
            logging.warning(f'Recognized: {text}')
