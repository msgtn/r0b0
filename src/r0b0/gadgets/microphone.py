from .gadget import Gadget, Message
from r0b0.utils.loaders import decode_msg
import logging, ast

import speech_recognition as sr

EVENTS = ["listen"]


# class Camera(Gadget):
class Microphone(
    Gadget,
):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)

        self.rec = sr.Recognizer()
        self.mic = self.get_target_microphone(
            microphone_name=config.get("microphone_name", None)
        )
        with self.mic as source:
            logging.warning("Calibrating microphone for ambience")
            self.rec.adjust_for_ambient_noise(source)
        self.handle_events(EVENTS)

    def get_target_microphone(self, microphone_name=None):
        mic = sr.Microphone()
        microphone_names = mic.list_microphone_names()
        if microphone_name is not None:
            if microphone_name in microphone_names:
                logging.warning(f"Found target microphone '{microphone_name}'.")
                mic = sr.Microphone(
                    device_index=microphone_names.index(
                        microphone_name)
                )
            else:
                logging.warning(f"Could not find target microphone '{microphone_name}'.")
        return mic


    @decode_msg
    def listen_event(self, data):
        msg = data["msg"]
        logging.warning(f"{self.name} now listening")
        with self.mic as source:
            res = self.rec.recognize_vosk(self.rec.listen(source))
            text = ast.literal_eval(res)["text"]
            self.emit(
                event="text",
                data={"event": "text", "text": text},
                namespace=self.namespace,
            )
            logging.warning(f"Recognized: {text}")
