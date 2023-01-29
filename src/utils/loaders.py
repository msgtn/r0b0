import os
import yaml
from src.config import CONFIG_DIR
from functools import partial
import pickle

def load_yaml(yaml_file: str):
    assert os.path.exists(yaml_file), f"No file {yaml_file}"
    with open(yaml_file,'r') as file:
        return yaml.load(file, Loader=yaml.Loader)
def load_config(config_name: str, config_type: str):
    return load_yaml(str(CONFIG_DIR / config_type / f"{config_name}.yaml"))
# wrappers
load_gadget = partial(load_config, config_type='gadgets')
load_rig = partial(load_config, config_type='rigs')

'''
Decorators for dumping and loading pickles
'''
def dump_pickle(func):
    def _inner_func(s,event,data,**kwargs):
        # print(s,event,data,kwargs)
        if data.get('msg',None) is not None:
            data['msg']=pickle.dumps(data['msg'])
        return func(s,event,data,**kwargs)
    return _inner_func
def load_pickle(func):
    def _inner_func(s,data,**kwargs):
        # print(s,data,kwargs)
        if data.get('msg',None) is not None:
            data['msg']=pickle.loads(data['msg'])
        return func(s,data,**kwargs)
            
    return _inner_func