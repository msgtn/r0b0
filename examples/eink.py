import os
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig
from r0b0.cables.eink_cables import SaveImageCable

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
    print(LOCALHOST)
    rig = Rig(
        # Default: https://localhost:8080
        hostname=LOCALHOST,
        port=SERVER_PORT,
        # Point to wherever you created the OpenSSL keys
        certfile=os.path.join(os.path.dirname(__file__), 'csr.pem'),
        keyfile=os.path.join(os.path.dirname(__file__), 'key.pem'),
        # pages_folder=os.path.abspath(os.path.join(os.path.dirname(__file__), '../pages/eink/eink/build/'))
        pages_folder=os.path.abspath(os.path.join(os.path.dirname(__file__), '../pages/eink/html/'))
    )

    # Create the gadgets
    eink_page = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, 'eink_page.yaml'))
    rig.add_cable(
        cable=SaveImageCable(),
        tx_gadget=eink_page)
    
    # Power on the rig
    print(rig._target)
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