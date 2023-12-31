import os
import yaml
from r0b0 import logging
from r0b0.config import CONFIG_DIR, LOCALHOST, SERVER_PORT

from functools import partial
import pickle

def load_yaml(yaml_file: str, **kwargs):
    """
    Load a yaml as a dictionary

    Arguments:
        yaml_file: The path to the yaml file

    Returns:
        yaml_dict: The dictionary loaded from the yaml

    Raises:
    """
    # check both 'yaml' and 'yml
    if not os.path.exists(yaml_file):
        yaml_file = yaml_file.replace('.yaml','.yml')
    assert os.path.exists(yaml_file), f"No file {yaml_file}"
    with open(yaml_file,'r') as file:
        yaml_dict = yaml.load(file, Loader=yaml.Loader)
        yaml_dict.update(kwargs)
        return yaml_dict


def load_config(config_name: str, config_type: str) -> dict:
    """
    Load a configuration from a yaml file in the config directory.


    Arguments:
        config_name: The name of the configuration to load
        config_type: The 
    """
    config_yaml = load_yaml(
        str(CONFIG_DIR / config_type / f"{config_name}.yaml"),
        )
    config_yaml.update(dict(
        name=config_name,
        config_type=config_type
    ))
    logging.debug(config_yaml)
    
    return config_yaml
# wrappers
load_gadget = partial(load_config, config_type='gadgets')
load_rig = partial(load_config, config_type='rigs')

'''
Decorators for dumping and loading pickles
'''
# Gadget.emit
def encode_msg(func):
    def _inner_func(s,event,data,**kwargs):
        # logging.debug(s)
        # logging.debug(event)
        # logging.debug(data)
        # logging.debug(kwargs)
        # if data.get('msg',None) is not None:
        if 'msg' in data:
            data['msg']=pickle.dumps(data['msg'])
        return func(s,event,data,**kwargs)
    return _inner_func

# Gadget handler
def decode_msg(func):
    def _inner_func(s,data,**kwargs):
        # logging.debug(s,data,kwargs)
        logging.debug(data)
        # print(data)
        if isinstance(data,dict) and data.get('msg',None) is not None:
            data['msg']=pickle.loads(data['msg'])
        return func(s,data,**kwargs)
    return _inner_func