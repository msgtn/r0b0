from pathlib import Path
ROOT_DIR = Path('./').absolute()
CONFIG_DIR = ROOT_DIR / 'config'
SRC_DIR = ROOT_DIR / 'src'
CABLES_DIR = SRC_DIR / 'cables'
GADGETS_DIR = SRC_DIR / 'gadgets'
KINEMATICS_DIR = SRC_DIR / 'kinematics'
UTILS_DIR = SRC_DIR / 'utils'

STATIC_DIR = GADGETS_DIR / 'static'
PUBLIC_DIR = GADGETS_DIR / 'public'

with open(str(ROOT_DIR / 'blossom_todo.md'),'r') as infile:
    with open(str(ROOT_DIR / 'blossom_todo_bak.md'),'w') as outfile:
        outfile.writelines(infile.readlines())