import logging
from r0b0.cables import Cable


class MPiCable(Cable):
    def __init__(self):
        self.input_event = "pi_button"
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
        button = data['button']

        if button not in self.button_funcs:
            logging.warning(f"Button {button} not assigned to a method")
        return {"event": data["button"]}
        # return {"event": "call_method", "method": self.button_funcs[button]}

        pass
