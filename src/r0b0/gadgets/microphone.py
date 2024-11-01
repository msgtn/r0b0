from .gadget import Gadget, Message
from r0b0.utils.loaders import decode_msg
import logging, ast

import speech_recognition as sr
from vosk import SetLogLevel

SetLogLevel(-1)

EVENTS = ["listen"]


# class Camera(Gadget):
class Microphone(
    Gadget,
):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)

        self.rec = sr.Recognizer()
        self.microphone_name = config.get("microphone_name", None)
        self.mic = self.get_target_microphone(
            microphone_name=self.microphone_name
        )
        self.timeout = config.get("timeout", 2)
        # with self.mic as source:
        #     logging.warning("Calibrating microphone for ambience")
#     self.rec.adjust_ufor_ambient_noise(source)
        self.handle_events(EVENTS)

    def get_target_microphone(self, microphone_name=None):
        mic = sr.Microphone()
        microphone_names = mic.list_microphone_names()
        if microphone_name is not None:
            if microphone_name in microphone_names:
                logging.warning(f"Found target microphone '{microphone_name}'.")
                mic = sr.Microphone(
                    device_index=microphone_names.index(microphone_name),
                    sample_rate=96000,
                )

            else:
                logging.warning(
                    f"Could not find target microphone '{microphone_name}'."
                )
        return mic

    def is_already_listening(self, source=None):
        with self.mic as source:
            if source.stream is not None:
                logging.warning("Microphone is already listening!")
                return True
        return False

    @decode_msg
    def listen_event(self, data):
        logging.warning("Microphone listen event.")
        msg = data["msg"]
        if self.mic.stream is not None:
            logging.warning("Microphone is already listening!")
            return
        # if self.is_already_listening():
        #     return

        # self.mic = self.get_target_microphone(self.microphone_name)
        with self.mic as source:
            
            def get_res():
                logging.warning(f"Calibrating {self.name} for ambient noise.")
                self.rec.adjust_for_ambient_noise(source)
                logging.warning(f"{self.name} now listening with timeout {self.timeout}s.")
                res = self.rec.recognize_vosk(
                    self.rec.listen(
                        source,
                        timeout=self.timeout
                        ))
                return res
            # if not self.is_already_listening():
            res = get_res()
            text = ast.literal_eval(res)["text"]
            # breakpoint()
            # self.mic.stream = None
            self.emit(
                event="text",
                data={"event": "text", "text": text},
                namespace=self.namespace,
            )
            logging.warning(f"Recognized: {text}")
    # with self.mic as source:
    #     breakpoint()
