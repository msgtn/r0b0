import logging
import numpy as np
from r0b0.gadgets.time_controller import TimeMode
from r0b0.cables import Cable
import io, os


class Motion2ModeCable(Cable):
    def __init__(
        self,
    ):
        super().__init__()
        self.input_event = "motor_motion"

    def __call__(self, data):
        # breakpoint()
        # print(data)
        # TODO - generalize the dictionary / Message that gets sent from velocity events
        data["value"] = data["dxl_motor"]
        if not data["value"]["moving"]:
            return
        # Filter small movements
        # if np.abs(data['value']<10):
        #     return
        if np.abs(data["value"]["velocity"]) < 50:
            return
        direction = "left" if data["value"]["velocity"] > 0 else "right"

        if direction == "left":
            mode = "stopwatch"
        else:
            mode = "timer"
        return {
            "event": "set_mode",
            "mode": mode,
            "position": data["value"]["position"],
        }


class Tick2MotionCable(Cable):
    def __init__(
        self,
    ):
        super().__init__()
        self.input_event = "tick"

    def __call__(self, data):
        direction = data["direction"]
        enable_event = {
            "event": "enable",
        }
        position_event = {
            "event": "position",
            "value": [data["position"]],
            # "motor_id": ["dxl_motor"],
            "motor_id": [1],
            "absolute": True,
        }
        # position_event = {
        #     "event": "position",
        #     "value": [direction*200],
        #     # "motor_id": ["dxl_motor"],
        #     "motor_id": [1],
        #     "absolute": False,
        # }
        disable_event = {"event": "disable"}
        return [
            enable_event,
            position_event,
            # disable_event
        ]


class Motion2DisableCable(Cable):
    def __init__(self):
        super().__init__()
        self.input_event = "position"

    def __call__(self, data):
        # print(self)
        return {"event": "disable"}


class Idle2ResetCable(Coble):
    def __init__(self):
        super().__init__()
        self.input_event = "idle"
        self.position_event = {
            "event": "position",
            "value": [0],
            "motor_id": [1],
            "absolute": True,
        }

    def __call__(self, data):
        return self.position_event


class Position2ModeCable(Cable):
    """
    After reset,
    dxl_motor.disable(); dxl_motor.POLL_MOVEMENT = True;

    :param Cable: _description_
    """

    def __init__(
        self,
    ):
        super().__init__()
        self.input_event = "position"
        self.start_position = 0
        self.last_position = None

    def __call__(self, data):
        # print("pos2mode", data)
        position = data["1"]["data"]
        if position > 3900 or position < 200:
            return {"event": "set_mode", "mode": "idle"}
        if not self.last_position:
            # self.start_position = position
            self.last_position = position
            # print(f"{self.start_position:}")
        else:
            if np.abs(self.last_position - position) > 1000:
                self.last_position = None
                return {"event": "set_mode", "mode": "idle"}
