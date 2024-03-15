from .gadget import Gadget, Message
from r0b0.utils.loaders import decode_msg
import logging

from threading import Thread
from aiortc import RTCPeerConnection, RTCSessionDescription
from functools import partial


class RTCGadget(Gadget):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.pcs = set()
        self.negotiate()
        # self.on('broadcaster',
        #     handler=partial(print, value='test'))
        pass

    def start(self):
        Thread.start(self)

        # to page?
        # send watcher

    def negotiate(self):
        # referencing controller.js and
        # https://github.com/aiortc/aiortc/blob/main/examples/webcam/webcam.py

        # broadcaster.js receives this
        self.on("broadcaster", handler=partial(self.emit, event="watcher", data={}))
        # self.emit(
        #     event='watcher',
        #     data={}
        # )
        logging.debug("emitted watcher")

        # handle candidate - maybe not necessary?
        # self.on('candidate',

        #     )

        # handle offer
        self.on("offer", handler=self.handle_offer)

        # send answer

    async def handle_offer(self, data):
        logging.debug(data)
        params = data["params"]
        # logging.debug(data)
        pc = RTCPeerConnection()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        self.pcs.add(pc)
        pc.setRemoteDescription(offer)

        answer = await pc.createAnswer()

        await pc.setLocalDescription(answer)
        self.emit(
            "answer",
            data={
                # 'sid':self.
                pc.localDescription,
            },
        )

        @pc.on("track")
        def on_track(track):
            logging.debug("received track")
