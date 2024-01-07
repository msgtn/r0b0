import os
import logging

logging.basicConfig(
    encoding='utf-8',
    # level=logging.DEBUG,
    level=logging.WARNING,
)
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig
from r0b0.cables.blsm import Motion2MotorCable


CONFIG_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),'../config/gadgets/'))

def main():
    # Start the server
    # print(LOCALHOST)
    with open(os.path.abspath(os.path.join(os.path.dirname(__file__), '../ngrok_public_url.txt')), 'r') as _file:
        socket_addr = _file.readlines()[0].strip()
    # print(LOCALHOST)
    
    rig = Rig(
        # Default: https://localhost:8080
        # hostname='0.0.0.0',
        hostname=LOCALHOST,
        port=SERVER_PORT,
        # Point to wherever you created the OpenSSL keys
        certfile=os.path.join(os.path.dirname(__file__), 'csr.pem'),
        keyfile=os.path.join(os.path.dirname(__file__), 'key.pem'),
        pages_folder=os.path.abspath(os.path.join(os.path.dirname(__file__), '../pages/blsm/')),
        socket_addr=socket_addr,
    )

    # Create the gadgets
    blsm_dxl = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, 'blsm_dxl.yaml'))
    blsm_phone = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, 'blsm_phone.yaml'))
    motion2motor_cable = Motion2MotorCable()

    rig.add_cable(
        cable=motion2motor_cable,
        rx_gadget=blsm_dxl,
        tx_gadget=blsm_phone,
        )

    test_emit_dict = {'event': 'position', 'data': {'event': 'position', 'msg': b'\x80\x04\x95y\x00\x00\x00\x00\x00\x00\x00\x8c\x13r0b0.gadgets.gadget\x94\x8c\x07Message\x94\x93\x94)\x81\x94}\x94(\x8c\x05event\x94\x8c\x08position\x94\x8c\x05value\x94]\x94(M/\x03M\x93\x06M\xd8\x06M\xd4\x07e\x8c\x08motor_id\x94]\x94(K\x01K\x02K\x03K\x04e\x8c\x08absolute\x94\x88ub.'}, 'to': None, 'include_self': False, 'namespace': '/blsm_dxl'}
    
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