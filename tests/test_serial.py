import pytest
import time
import serial
import random
from r0b0.gadgets.serial_gadget import SerialGadget

@pytest.fixture
def ser():
    return SerialGadget()

class TestSerial:
    # def test_init(self, ser):
    #     breakpoint()

    # def test_listening_thread(self, ser):
    #     print("Listening test")
    #     ser.listen()
    #     breakpoint()
    #     time.sleep(5)
    #     ser.listen(False)
    #     breakpoint() 

    def test_write(self, ser):
        # ser.write("testing\n", baudrate=10_000_000, write_timeout=2)
        opts = [
            "IS THIS IT",
            "TURN ON THE BRIGHT LIGHTS",
            "FEVER TO TELL",
        ]
        long_string = """
Zankoku na tenshi no you ni
Shounen yo, shinwa ni nare...

Aoi kaze ga ima mune no doa wo tataitemo,
Watashi dake wo tada mitsumete
Hohoenderu Anata
Sotto Fureru mono
Motomeru koto ni muchuu de,
Unmei sae mada shiranai itaike na hitomi

Dakedo itsuka kizuku deshou
Sono senaka ni wa
Haruka mirai mezasu tame no
Hane ga aru koto...

Zankoku na tenshi no tēze
Madobe kara yagate tobitatsu
Hotobashiru atsui patosu de
Omoide wo uragiru nara
Kono sora wo daite kagayaku
Shounen yo, shinwa ni nare
"""
        long_string = long_string.replace("\n", " ")
        # ser.write("print_text('this is finally working?')", baudrate=10_000_000, write_timeout=2)
        # ser.write("print_text('IS THIS IT')", baudrate=10_000_000, write_timeout=2)
        ser.write("\x03\r\f", baudrate=10_000_000, write_timeout=2)
        time.sleep(1)
        blsm = " "*16*3 + " "*6 + "blsm"
        # ser.write(f"print_text('{random.choice(opts)}')\r\f", baudrate=10_000_000, write_timeout=2)
        # ser.write(f"print_text('{blsm}')\r\f", baudrate=10_000_000, write_timeout=2)
        ser.write(f"print_text('{long_string}')\r\f", baudrate=10_000_000, write_timeout=2)
        time.sleep(5)
        ser.write(f"main()\r\f", baudrate=10_000_000, write_timeout=2)
        # breakpoint()