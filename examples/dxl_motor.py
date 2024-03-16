import os
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig

import logging

logging.basicConfig(
    encoding='utf-8',
    level=logging.DEBUG
)

def main():
    # Create the gadgets
    dxl_motor = r0b0.gadgets.from_config(
        os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),'../config/gadgets/dxl_motor.yaml')))
    dxl_motor.disable()
    # for motor in dxl_motor.dxl_dict.values():
    #     motor.disable()
    dxl_motor.poll_motion = True
    dxl_motor.moving_thread.start()
    motor = dxl_motor.dxl_dict['dxl_motor']
    try:
        # Serve indefinitely 
        breakpoint()
    except KeyboardInterrupt:
        # Power off the rig
        exit()


if __name__=="__main__":
    main()