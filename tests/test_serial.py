import pytest
import time
import serial
from r0b0.gadgets.serial_gadget import SerialGadget

@pytest.fixture
def ser():
    return SerialGadget()

class TestSerial:
    # def test_init(self, ser):
    #     breakpoint()

    def test_listening_thread(self, ser):
        print("Listening test")
        ser.listen()
        # breakpoint()
        time.sleep(5)
        ser.listen(False)
        # breakpoint() 