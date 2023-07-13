from pypot.dynamixel import Dxl320IO
from r0b0 import logging

if __name__=="__main__":
    dxl = Dxl320IO(
        port='/dev/tty.usbmodem212401',
        baudrate=1e6,
    )
    print(f'Found motors {dxl.scan(range(20))}')
    breakpoint()