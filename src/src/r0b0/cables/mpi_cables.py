import logging
from r0b0.cables import cable


class MPiCable(Cable):
    def __init__(self):
        # Maps buttons to functions to call
        self.button_funcs = {
            "d_down": "shutter15",
            "d_right": "shutter60",
            "d_up": "shutter250",
            "d_left": "shutter1000",
            "shutter": "release_shutter",
        }
        pass

    def __call__(self, data):
        super().__call__(data)

        if data.button not in self.button_funcs:
            logging.warning(f"Button {data.button} not assigned to a method")
        return {"event": "call_method", "method": self.button_funcs[data.button]}

        pass
