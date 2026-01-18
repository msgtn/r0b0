import logging
import numpy as np
from r0b0.cables import Cable
from PIL import Image
import io, os


class SaveImageCable(Cable):
    def __init__(
        self,
    ):
        super().__init__()
        self.input_event = "file_upload"

    def __call__(self, data):
        print("Received image")

        image_stream = io.BytesIO(data["image"])
        image = Image.open(image_stream)
        image.save(os.path.expanduser("~/tmp_image.jpg"))


class Upload2DrawCable(Cable):
    """
    Page file_upload to E-Ink draw_image
    """

    def __init__(
        self,
    ):
        super().__init__()
        self.input_event = "file_upload"

    def __call__(self, data):
        return {"event": "draw_image", "image": data["image"]}
