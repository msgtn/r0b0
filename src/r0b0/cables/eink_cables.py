from r0b0 import logging
import numpy as np
from r0b0.cables import Cable
from PIL import Image
import io, os

class SaveImageCable(Cable):
    """
    Converts phone's device motion into motor positions for Blossom
    """
    def __init__(self,):
        # super().__init__()
        self.input_event = 'file_upload'

    def __call__(self, data):
        print("Received image")
        # print(data)
        image_stream = io.BytesIO(data['image'])
        image = Image.open(image_stream)
        image.save(os.path.expanduser('~/tmp_image.jpg'))
        # return {
        #     'event':'position',
        #     'value':device_motion2dxl_motor(data),
        #     'motor_id':[1,2,3,4],
        #     'absolute':True
        # }
