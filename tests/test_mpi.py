import pytest

from r0b0.gadgets import PiCamera
picam = PiCamera()

class TestPiCamera():
    @pytest.mark.parametrize("shutter_speed", [1e6/15, 1e6/30, 1e6/1000])
    def test_set_shutter_speed(self, shutter_speed):
        picam.set_shutter_speed(shutter_speed)
        assert picam.shutter_speed