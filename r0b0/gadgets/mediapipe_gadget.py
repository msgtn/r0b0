from .gadget import Gadget, Message
from r0b0.utils.loaders import load_msg
from r0b0 import logging

import cv2
from cv2 import VideoCapture

EVENTS = [
    'read'
]

# class Camera(Gadget):
class Camera(Gadget, VideoCapture):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self,config,**kwargs)
        VideoCapture.__init__(self,config['index'])
        
        self.handle_events(EVENTS)
        
    def get_frame(self):
        # self.
        ret,frame = self.read()
        if ret:
            return frame
        else:
            logging.warning(
                f'Camera gadget {self.name} not get frame')
            return None
        pass