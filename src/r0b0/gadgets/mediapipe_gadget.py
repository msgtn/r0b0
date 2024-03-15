from .gadget import Gadget, Message
from r0b0.utils.loaders import decode_msg
import logging

import mediapipe as mp

mp_pose = mp.solutions.pose
from mp.solutions.pose import Pose as MediaPipePose

EVENTS = ["read"]


class MediaPipeGadget(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        pass


class MediaPipePoseGadget(MediaPipeGadget, MediaPipePose):
    def __init__(self, config, **kwargs):
        MediaPipeGadget.__init__(self, config, **kwargs)
        MediaPipePose.__init__(
            self,
            **{
                k: config[k]
                for k in ["min_detection_confidence", "min_tracking_confidence"]
            }
        )
        self.on("process", handler=self.process_event, namespace=self.namespace)

    @decode_msg
    def process_event(self, data):
        msg = data["msg"]
        results = self.process()
        breakpoint()
        self.emit(
            event="process",
            data={
                "event": "process",
            },
        )
