

import os, pickle, time
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig
import socketio
from socketio import Client
from r0b0.cables.cable import Key2TimeModeCable

import logging
logging.basicConfig(
    encoding='utf-8',
    level=logging.DEBUG
)
CONFIG_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),'../config/gadgets/'))

def main():


    with open(os.path.abspath(os.path.join(os.path.dirname(__file__), '../ngrok_public_url.txt')), 'r') as _file:
        socket_addr = _file.readlines()[0].strip()
    # Start the server
    rig = Rig(
        # Default: https://localhost:8080
        hostname=LOCALHOST,
        port=SERVER_PORT,
        # Point to wherever you created the OpenSSL keys
        certfile=os.path.join(os.path.dirname(__file__), 'csr.pem'),
        keyfile=os.path.join(os.path.dirname(__file__), 'key.pem'),
        pages_folder=os.path.abspath(os.path.join(os.path.dirname(__file__), '../pages/blsm/')),
        socket_addr=socket_addr,
    )
    print(rig._target)

    # Create the gadgets
    pygame_keys = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, 'pygame_keys.yaml'))
    tc = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, 'time_controller.yaml'))
    key2mode_cable = Key2TimeModeCable()
    rig.add_cable(tx_gadget=pygame_keys, rx_gadget=tc, cable=key2mode_cable)
    breakpoint()
    # Power on the rig
    rig.power_on()

    breakpoint()
    print(tc.sid, pygame_keys.sid)
    try:
        # input()
        # Serve indefinitely 
        # test_emit()

        breakpoint()
    except KeyboardInterrupt:
        # Power off the rig
        rig.power_off()
        exit()


if __name__=="__main__":
    main()