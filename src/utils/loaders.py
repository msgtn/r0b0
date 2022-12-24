import os
import yaml
from src.config import CONFIG_DIR

def load_yaml(yaml_file: str):
    assert os.path.exists(yaml_file), f"No file {yaml_file}"
    print(yaml_file)
    with open(yaml_file,'r') as file:
        return yaml.load(file, Loader=yaml.Loader)
        
def load_config(config_name: str):
    return load_yaml(str(CONFIG_DIR / f"{config_name}.yaml"))