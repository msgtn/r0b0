from pypot.dynamixel import Dxl320IO
import logging

if __name__ == "__main__":
    dxl = Dxl320IO(
        port="/dev/tty.usbmodem212401",
        baudrate=1e6,
    )
    print(f"Found motors {dxl.scan(range(20))}")
    breakpoint()
    """
    To change IDs:
    dxl.change_id({
        old_id_1:new_id_1,
        old_id_2:new_id_2,
    })
    """
