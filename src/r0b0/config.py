from os.path import dirname
from pathlib import Path
# ROOT_DIR = Path('./').absolute()
ROOT_DIR = Path(dirname(__file__)).absolute()
CONFIG_DIR = ROOT_DIR / 'config'
TAPES_DIR = ROOT_DIR / 'tapes'
SRC_DIR = ROOT_DIR / 'r0b0'
CABLES_DIR = SRC_DIR / 'cables'
GADGETS_DIR = SRC_DIR / 'gadgets'
KINEMATICS_DIR = SRC_DIR / 'kinematics'
UTILS_DIR = SRC_DIR / 'utils'
MESSAGES_DIR = SRC_DIR / 'messages'
CABLES_DIR = SRC_DIR / 'cables'
BROWSER_DIR= SRC_DIR / 'browser'

CSR_PEM = SRC_DIR / 'csr.pem'
KEY_PEM = SRC_DIR / 'key.pem'

STATIC_DIR = GADGETS_DIR / 'static'
PUBLIC_DIR = GADGETS_DIR / 'public'

LOCALHOST = 'localhost'
MIDI_HOST = LOCALHOST
ROBOT_HOST = LOCALHOST
SERVER_PORT = 8080
MIDI_PORT = 9000
ROBOT_PORT = 9090
