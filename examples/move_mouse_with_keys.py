import os
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig
from r0b0.cables.cable import Key2MouseCable

def main():
    # Start the server
    # r0b0.init()
    rig = Rig(
        hostname=LOCALHOST,
        port=SERVER_PORT,
        certfile=os.path.join(os.path.dirname(__file__), 'csr.pem'),
        keyfile=os.path.join(os.path.dirname(__file__), 'key.pem'),
    )

    # Create the gadgets
    pygame_keys = r0b0.gadgets.from_config(os.path.abspath('../config/gadgets/pygame_keys.yaml'))
    mouse = r0b0.gadgets.from_config(os.path.abspath('../config/gadgets/mouse.yaml'))
    key2mouse_cable = Key2MouseCable()

    rig.add_cable(
        tx_gadget=pygame_keys,
        rx_gadget=mouse,
        cable=key2mouse_cable,
        )
    rig.power_on()
    breakpoint()

if __name__=="__main__":
    main()