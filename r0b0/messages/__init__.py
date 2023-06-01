import importlib
import glob
from os.path import basename
from r0b0 import logging
from r0b0.config import MESSAGES_DIR
from r0b0.messages.msg_funcs import *

'''
This will import everything found
in python files in this r0b0.messages director

Usage:
from r0b0.messages import *
'''

for file in glob.glob(str(MESSAGES_DIR / '*.py')):
    if '__init__' in file: continue
    file_package = basename(file).split('.')[0]
    logging.debug(file_package)
    mod = importlib.import_module(
        f'r0b0.messages.{file_package}',
        '*')
    # from mod import *
    # importlib.import_module('r0b0.messages','asdfd')
    
# breakpoint()