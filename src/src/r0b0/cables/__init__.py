import importlib
import glob
import os
from os.path import basename, dirname
import logging

logger = logging.getLogger(__name__)

from r0b0.config import MESSAGES_DIR, CABLES_DIR

# from r0b0.cables.msg_funcs import *
# print(f'abs: {abspath(".")}, rel: {relpath(".")}')

"""
This will import everything found
in python files in this r0b0.messages director

Usage:
from r0b0.messages import *
"""

# cable_files = sorted(glob.glob(str(CABLES_DIR / '*.py')))
cable_files = sorted(glob.glob(os.path.join(dirname(__file__), "*.py")))
for file in cable_files:
    if "__init__" in file:
        continue
    mod_name = basename(file).split(".")[0]
    logging.debug(mod_name)
    try:
        mod = importlib.import_module(f"r0b0.cables.{mod_name}")
        globals().update({mod_name: mod})
        globals().update({v: getattr(mod, v) for v in dir(mod) if "__" not in v})
    except:
        logging.warning(
            f"Could not import cables from {file}; dependencies may not have been installed."
        )
