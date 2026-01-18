import numpy as np
from r0b0.cables import Cable


def key2mouse_place(data=None):
    if data is None:
        return {"event": "keydown"}
    key2pos_dict = {
        "q": [100, 100],
        "w": [500, 100],
        "e": [900, 100],
        "a": [100, 400],
        "s": [500, 400],
        "d": [900, 400],
        "z": [100, 700],
        "x": [500, 700],
        "c": [900, 700],
    }
    # key2pos_dict.setdefault([500,400])
    [x, y] = key2pos_dict.get(data["unicode"], [500, 400])
    return {
        "event": "mouse_place",
        "x": x,
        "y": y,
    }


class Motor2MouseCable(Cable):
    """
    Converts key presses to absolute mouse positions
    """

    def __init__(
        self,
    ):
        self.input_event = "motor_velocity"

    def __call__(self, data):
        # print(data)
        axis = 0
        # TODO this is a kludge
        value = data["dxl_motor"]
        # print(axis,value)
        if np.abs(value) < 10.0:
            value = 0
        value /= -100.0
        return {
            "event": "mouse_move",
            "axis": 0,
            "value": value,
        }
