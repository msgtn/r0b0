import os
import logging
# logging.config.dictConfig()
logging.basicConfig(
    encoding='utf-8',
    level=logging.DEBUG,
    # level=logging.WARNING,
)
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig
from r0b0.cables.cable import Key2MouseCable

# logger = logging.getLogger(__name__)
# logger.setLevel(logging.DEBUG)

def main():
    """Example rig that maps keys to absolute mouse positions.
    Use the keys [[Q,W,E],[A,S,D],[Z,X,C]] to move the mouse.
    Note that the PyGame window that starts up *must* be focused for the program to register the key presses.
    """
    # Start the rig
    rig = Rig(
        # Default: https://localhost:8080
        hostname=LOCALHOST,
        port=SERVER_PORT,
        # Point to wherever you created the OpenSSL keys
        certfile=os.path.join(os.path.dirname(__file__), 'csr.pem'),
        keyfile=os.path.join(os.path.dirname(__file__), 'key.pem'),
    )

    # Create the gadgets and cable
    pygame_keys = r0b0.gadgets.from_config(os.path.abspath('./config/gadgets/pygame_keys.yaml'))
    mouse = r0b0.gadgets.from_config(os.path.abspath('./config/gadgets/mouse.yaml'))
    key2mouse_cable = Key2MouseCable()

    # Add the cable to the rig
    rig.add_cable(
        tx_gadget=pygame_keys,
        rx_gadget=mouse,
        cable=key2mouse_cable,
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