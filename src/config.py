from pathlib import Path
ROOT_DIR = Path('./').absolute()
CONFIG_DIR = ROOT_DIR / 'config'
TAPES_DIR = ROOT_DIR / 'tapes'
SRC_DIR = ROOT_DIR / 'src'
CABLES_DIR = SRC_DIR / 'cables'
GADGETS_DIR = SRC_DIR / 'gadgets'
KINEMATICS_DIR = SRC_DIR / 'kinematics'
UTILS_DIR = SRC_DIR / 'utils'

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

with open(str(ROOT_DIR / 'blossom_todo.md'),'r') as infile:
    with open(str(ROOT_DIR / 'blossom_todo_bak.md'),'w') as outfile:
        outfile.writelines(infile.readlines())