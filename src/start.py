import sys
import yaml
from src.config import *

def main():
    pass

def load_robot_config(robot_name):
    with open(str(CONFIG_DIR / f'{robot_name}.yaml'),'r') as yaml_file:
        return yaml.load(yaml_file, Loader=yaml.FullLoader)

if __name__=="__main__":
    params = ['robot_name']
    assert len(sys.argv)>1, "No robot name provided"
    # breakpoint()
    params = {p:v for p,v in zip(params,sys.argv[1:])}
    robot_config = load_robot_config(params['robot_name'])
    breakpoint()
    main(sys.argv[1])