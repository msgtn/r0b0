import os
import pytest
import r0b0
from r0b0.rigs import Rig
from r0b0.config import LOCALHOST, SERVER_PORT, CONFIG_DIR


with open(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../ngrok_public_url.txt')), 'r') as _file:
    socket_addr = _file.readlines()[0].strip()
_rig = Rig(
        # Default: https://localhost:8080
        # hostname='0.0.0.0',
        hostname=LOCALHOST,
        port=SERVER_PORT,
        # Point to wherever you created the OpenSSL keys
        certfile=os.path.join(os.path.dirname(__file__), 'csr.pem'),
        keyfile=os.path.join(os.path.dirname(__file__), 'key.pem'),
        pages_folder=os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../pages/blsm')),
        # socket_addr=f'https://{LOCALHOST}:{SERVER_PORT}/'
        # socket_addr=socket_addr,
    )

CONFIG_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),'../../../config/gadgets/'))
blsm_phone = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, 'blsm_phone.yaml'))
_rig.add_gadget(blsm_phone)
@pytest.fixture
def rig():
    return _rig