import numpy as np


class ExponentialFilter:
    def __init__(self, alpha: float = 0.5):
        self.alpha = alpha
        self.value = None

    # TODO: typehint with typevars
    def __call__(self, value):
        if self.value is None:
            self.value = value
        else:
            self.value = (self.alpha * value) + ((1 - self.alpha) * self.value)
        return self.value
