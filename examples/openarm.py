import os
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig
from r0b0.cables.cable import MidiRel2PositionCable

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
    op1 = r0b0.gadgets.from_config(os.path.abspath('../config/gadgets/op1.yaml'))
    open_arm = r0b0.gadgets.from_config(os.path.abspath('../config/gadgets/open_arm.yaml'))
    midi_rel2position_cable = MidiRel2PositionCable()

    rig.add_cable(
        cable=midi_rel2position_cable,
        rx_gadget=open_arm,
        tx_gadget=op1,
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