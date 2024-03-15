#!/usr/bin/python
# -*- coding:utf-8 -*-
import sys
import os

import logging
from waveshare_epd.epd2in7 import EPD
import time
from PIL import Image, ImageDraw, ImageFont
import io
import traceback

from .gadget import Gadget, Message, logging
from r0b0.utils.loaders import decode_msg, encode_msg

EVENTS = ["draw_image"]


def change_contrast(img, level):
    factor = (259 * (level + 255)) / (255 * (259 - level))

    def contrast(c):
        value = 128 + factor * (c - 128)
        return max(0, min(255, value))

    return img.point(contrast)


class EInk(Gadget, EPD):
    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, **kwargs)
        self.handle_events(EVENTS)
        EPD.__init__(self)
        EPD.init(self)
        EPD.Clear(self, 0xFF)
        # self = epd2in7()
        # self.init()
        # self.Clear(0xFF)

    @decode_msg
    def draw_image_event(self, data):
        msg = data["msg"]
        logging.warning("warning")
        logging.debug("Received Image")
        Himage = Image.new("1", (self.height, self.width), 255)  # 255: clear the frame
        if not isinstance(msg.image, bytes):
            with open(os.path.expanduser("~/image.txt"), "w") as _file:
                _file.write(msg.image)
            msg.image = bytes(msg.image, "utf-8")
        # print(msg.image)
        image_stream = io.BytesIO(msg.image)
        image = Image.open(image_stream)
        image = change_contrast(image, 200)
        image = image.convert("1")
        image = image.rotate(180)
        image = image.resize((self.height, self.width))
        draw = ImageDraw.Draw(image)

        self.display(self.getbuffer(image))

        pass
