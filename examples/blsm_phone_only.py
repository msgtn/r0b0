import os

import logging

logging.basicConfig(
    encoding='utf-8',
    level=logging.DEBUG
)
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig

CONFIG_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),'../config/gadgets/'))

def main():
    # Start the server
    rig = Rig(
        # Default: https://localhost:8080
        hostname='0.0.0.0',
        port=SERVER_PORT,
        # Point to wherever you created the OpenSSL keys
        certfile=os.path.join(os.path.dirname(__file__), 'csr.pem'),
        keyfile=os.path.join(os.path.dirname(__file__), 'key.pem'),
        pages_folder=os.path.abspath(os.path.join(os.path.dirname(__file__), '../pages/blsm'))
    )
    print(rig._target)

    # Create the gadgets
    blsm_phone = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, 'blsm_phone.yaml'))
    rig.add_gadget(blsm_phone)
    print(blsm_phone._target)
    
    # Power on the rig
    rig.power_on()
    try:
        # input()
        # Serve indefinitely 
        breakpoint()
    except KeyboardInterrupt:
        # Power off the rig
        rig.power_off()
        exit()


if __name__=="__main__":
    main()