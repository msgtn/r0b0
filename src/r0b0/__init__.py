import os, glob
import logging as logging
logging.basicConfig(
    encoding='utf-8',
    # level=logging.DEBUG,
    level=logging.WARNING,
    )
import inspect

import time
get_timestamp = lambda: time.strftime('%Y%m%d%H%M%S')

# modules = glob.glob(os.path.join(os.path.dirname(__file__), '*', '__init__.py'))
# __all__ = [os.path.dirname(module) for module in modules]
# from . import cables, gadgets,rigs

import pkgutil

__all__ = []
modules = glob.glob(os.path.join(os.path.dirname(__file__), '*', '__init__.py'))
# for module in modules:
if True:
    # for loader, module_name, is_pkg in pkgutil.walk_packages(__path__):
    for loader, module_name, is_pkg in pkgutil.iter_modules(__path__):
    # for loader, module_name, is_pkg in pkgutil.walk_packages(os.path.dirname(module)):
        print(loader,module_name,is_pkg)
        __all__.append(module_name)
        _module = loader.find_module(module_name).load_module(module_name)
        globals()[module_name] = _module

# __all__ = ['cables','gadgets','rigs']