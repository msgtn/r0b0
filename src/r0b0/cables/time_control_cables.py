import logging
import numpy as np
from r0b0.cables import Cable
import io, os

class Motion2ModeCable(Cable):
    def __init__(self,):
        super().__init__()
        self.input_event = "motor_velocity"

    def __call__(self, data):
        # breakpoint()
        # TODO - generalize the dictionary / Message that gets sent from velocity events
        data['value'] = data['dxl_motor']
        direction = 'left' if data['value'] > 0 else 'right'

        if direction == 'left':
            mode = 'stopwatch'
        else:
            mode = 'timer'
        return {
            "event": "set_mode",
            "mode": mode,
        }
        

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
