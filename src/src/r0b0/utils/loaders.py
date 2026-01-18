import os
import yaml
# import logging
# logging = logging.getLogger(__name__)
from r0b0.config import CONFIG_DIR, LOCALHOST, SERVER_PORT
from r0b0 import logging

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
        yaml_file = yaml_file.replace(".yaml", ".yml")
    assert os.path.exists(yaml_file), f"No file {yaml_file}"
    with open(yaml_file, "r") as file:
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
    config_yaml.update(dict(name=config_name, config_type=config_type))
    logging.debug(config_yaml)

    return config_yaml


# wrappers
load_gadget = partial(load_config, config_type="gadgets")
load_rig = partial(load_config, config_type="rigs")

"""
Decorators for dumping and loading pickles
"""


# Gadget.emit
def encode_msg(func):
    """Decorator to encode a message with pickle to send
    non-serializable objects through sockets as strings.

    :param func: A function that emits an event through a socket
    """

    def _inner_func(s, event, data, *args, **kwargs):
        if "msg" in data:
            data["msg"] = pickle.dumps(data["msg"]).hex()
        return func(s, event, data, *args, **kwargs)

    return _inner_func


# Gadget handler
def decode_msg(func):
    """Decorator to decode a message with pickle.
    Loads the message into its original Message class.
    The data['msg'] value will probably be a hex-encoded string.
    The Message type should be installed in the environment.

    :param func: The handler function (usually a Gadget function)
    """

    logging.debug(f"Decoding message for {func}")
    def _inner_func(s, data, *args, **kwargs):

        logging.debug(f"inner_func data {data}")
        logging.debug(f"inner_func args {args}")
        logging.debug(f"inner_func kwargs {kwargs}")
        if isinstance(data, dict) and "msg" in data:

            # Load from a hex-encoded string
            # TODO - messages *should* be hex encoded if it is sent
            # from a function decorated with encode_msg
            if not isinstance(data["msg"], bytes):
                data["msg"] = bytes.fromhex(data["msg"])
            data["msg"] = pickle.loads(data["msg"])
            # print('after',data['msg'])

        # print(f'decoded data, {data}')

        return func(s, data, **kwargs)

    return _inner_func
