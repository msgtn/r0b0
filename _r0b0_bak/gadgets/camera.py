from .gadget import Gadget, Message
from r0b0.utils.loaders import decode_msg
import logging

import cv2
from cv2 import VideoCapture

EVENTS = ["read"]


# class Camera(Gadget):
class Camera(Gadget, VideoCapture):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        VideoCapture.__init__(self, config["index"])

        self.handle_events(EVENTS)

    def get_frame(self):
        # self.
        ret, frame = self.read()
        if ret:
            return frame
        else:
            logging.warning(f"Camera gadget {self.name} not get frame")
            return None
        pass

    @decode_msg
    def read_event(self, data):
        frame = self.get_frame()
        logging.warning(data)
        msg = data["msg"]
        logging.warning(msg.__dict__)
        if frame is not None:
            self.emit(
                event="frame",
                data={
                    "event": "frame",
                    "msg": self.get_frame(),
                },
                namespace=self.namespace,
            )
            if getattr(msg, "save", False):
                logging.warning("saving frame")
                cv2.imwrite("frame.jpg", frame)
            logging.warning("got frame")
        else:
            logging.warning(f"Camera gadget {self.name} not get frame")
