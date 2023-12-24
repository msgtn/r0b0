import os
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig
from r0b0.cables.cable import Motion2MotorCable

import logging
logging.basicConfig(
    encoding='utf-8',
    level=logging.DEBUG
)

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
    blsm_dxl = r0b0.gadgets.from_config(os.path.abspath('../config/gadgets/blsm_dxl.yaml'))
    blsm_phone = r0b0.gadgets.from_config(os.path.abspath('../config/gadgets/blsm_phone.yaml'))
    motion2motor_cable = Motion2MotorCable()

    rig.add_cable(
        cable=motion2motor_cable,
        rx_gadget=blsm_dxl,
        tx_gadget=blsm_phone,
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