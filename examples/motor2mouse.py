import os
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig
from r0b0.cables.mouse_funcs import Motor2MouseCable

import logging
logging.basicConfig(
    encoding='utf-8',
    level=logging.DEBUG
)
CONFIG_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),'../config/gadgets/'))

def main():
    # Start the server
    rig = Rig(
        # Default: https://localhost:8080
        hostname=LOCALHOST,
        port=SERVER_PORT,
        # Point to wherever you created the OpenSSL keys
        certfile=os.path.join(os.path.dirname(__file__), 'csr.pem'),
        keyfile=os.path.join(os.path.dirname(__file__), 'key.pem'),
    )
    # Create the gadgets
    
    dxl_motor = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, 'dxl_motor.yaml'))
    mouse = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, 'mouse.yaml'))
    motor2mouse_cable = Motor2MouseCable()
    dxl_motor.disable()
    # for motor in dxl_motor.dxl_dict.values():
    #     motor.disable()
    dxl_motor.poll_motion = True
    dxl_motor.moving_thread.start()
    
    rig.add_cable(
        cable=motor2mouse_cable,
        tx_gadget=dxl_motor,
        rx_gadget=mouse,
        )
    
    # Power on the rig
    rig.power_on()
    try:
        # Serve indefinitely 
        breakpoint()
    except KeyboardInterrupt:
        # Power off the rig
        rig.power_off()
        exit()


if __name__=="__main__":
    main()