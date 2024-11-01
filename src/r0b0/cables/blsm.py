import logging
import numpy as np
from r0b0.cables import Cable
from r0b0.kinematics.blsm import (
    device_motion2dxl_motor,
    device_motion2dxl_motor320,
    device_motion2arduino_motor,
)


class Motion2MotorCable(Cable):
    """
    Converts phone's device motion into motor positions for Blossom
    """

    def __init__(
        self,
    ):
        # super().__init__()
        self.input_event = "device_motion"

    def __call__(self, data):
        super().__call__(data)
        return {
            "event": "position",
            "value": device_motion2dxl_motor(data),
            "motor_id": [1, 2, 3, 4],
            "absolute": True,
        }


class Microphone2PromptCable(Cable):
    def __init__(self):
        self.input_event = "text"

    def __call__(self, data):
        super().__call__(data)
        # print("mic2prompt")
        logging.debug(f"Prompt: {data['text']}")
        return {"event": "prompt", "prompt_string": data["text"]}


class Text2PromptCable(Cable):
    def __init__(self):
        self.input_event = "phone_text"

    def __call__(self, data):
        super().__call__(data)
        # print("mic2prompt")
        # logging.info(f"Prompt: {data['text']}")
        logging.debug(f"Prompt: {data['text']}")
        return {"event": "prompt", "prompt_string": data["text"]}


class Microphone2StopctrlCable(Cable):
    def __init__(self):
        self.input_event = ""


def key2mic(data=None):
    if data is None:
        return {"event": "keydown"}
    if data["unicode"] == "m":
        return {
            "event": "listen",
        }


class Key2MicCable(Cable):
    def __init__(self):
        self.input_event = "keydown"

    def __call__(self, data):
        super().__call__(data)
        if data["unicode"] == "m":
            return {
                "event": "listen",
            }


def motion2motor(data=None):
    if data is None:
        return {"event": "device_motion"}
    # logging.debug(f'motion2motor {data}')
    logging.debug(device_motion2dxl_motor(data))
    return {
        "event": "position",  # 'value': # the function that gives
        "value": device_motion2dxl_motor(data),
        "motor_id": [1, 2, 3, 4],
        "absolute": True,
    }


def motion2motor320(data=None):
    if data is None:
        return {"event": "device_motion"}
    # logging.debug(f'motion2motor {data}')
    logging.debug(device_motion2dxl_motor320(data))
    return {
        "event": "position",  # 'value': # the function that gives
        "value": device_motion2dxl_motor320(data),
        "motor_id": [1, 2, 3, 4],
        "absolute": True,
    }


def motion2arduino_motor(data=None):
    if data is None:
        return {"event": "device_motion"}
    # logging.debug(f'motion2motor {data}')
    return {
        "event": "position",  # 'value': # the function that gives
        "value": device_motion2arduino_motor(data),
        # changes these to string keys instead of int ids
        "motor_id": [10, 6, 5, 9],
        "absolute": True,
    }


def joy2rover(data=None):
    """Converts joystick axis data to wheel motors for "roving" """
    if data is None:
        return {"event": "joyaxismotion"}

    axis = data["axis"]
    # logging.debug(data)
    if axis != 1:
        return {"event": "velocity", "motor_id": [], "value": []}

    velocity = int(data["value"] * 1000)
    if np.abs(velocity) < 100:
        velocity = 0

    return {
        "event": "velocity",
        "motor_id": [7, 8],
        "value": [velocity, -velocity],
        "absolute": False,
    }


def motion2velocity(data=None):
    if data is None:
        return {"event": "device_motion"}
    # logging.debug(f'motion2motor {data}')
    return {
        "event": "position",  # 'value': # the function that gives
        "value": device_motion2dxl_motor(data),
        "motor_id": [1, 2, 3, 4],
        # 'motor_kwargs':
        # 'absolute':True
    }


def joy2vel(data=None):
    if data is None:
        return {"event": "joyaxismotion"}
    motor_id, value = [], []
    if data["axis"] == 1:
        motor_id = 1
        value = int(data["value"] * 2000)
        if np.abs(value) < 200:
            value = 0
    return {
        "event": "velocity",
        "motor_id": motor_id,
        "value": value,
        "absolute": True,
    }


def response2blsm(data=None):
    if data is None:
        return {"event": "response"}
    logging.warning(data)
    res = data["response"]
    # blsm_tape = f"blsm_{res.split('.')[0].lower()}"

    # logging.warning(tape)
    return {"event": "play", "tape_name": f"blsm_{res.split('.')[0].lower()}"}


class Serial2PoseCable(Cable):
    def __init__(self):
        self.input_event = "serial"

    def __call__(self, data):
        super().__call__(data)
        if data["detected"]:
            pose_event = {
                "event": "position",
                "value": [0, 2000, 0, 1000],
                "motor_id": [1, 2, 3, 4],
                "absolute": True,
            }
            return [
                {"event": "stopControl", "namespace": "/"},
                {"event": "stop", "namespace": "/"},
                # {"event": "stopControl", "namespace": "/blsm_phone"},
            ] + [pose_event]*3


class Text2PoseCable(Cable):
    def __init__(self):
        self.input_event = "text"

    def __call__(self, data):
        # TODO - could make this a staticmethod decorator?
        super().__call__(data)
        return [
            {"event": "stop", "namespace": "/"},
            {
                "event": "position",
                "value": [0, 800, 800, 2000],
                "motor_id": [1, 2, 3, 4],
                "absolute": True,
            },
        ]


class Response2PoseCable(Cable):
    def __init__(self):
        self.input_event = "response"

    def __call__(self, data):
        super().__call__(data)
        return {
            "event": "position",
            "value": [500, 500, 500, 2000],
            "motor_id": [1, 2, 3, 4],
            "absolute": True,
        }

class Response2TypeCable(Cable):
    def __init__(self):
        self.input_event = "response"

    def __call__(self, data):
        super().__call__(data)
        return {
            "event": "write",
            "text": data["text"]
        }