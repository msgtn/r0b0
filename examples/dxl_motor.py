import os, random, time
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig

import logging

logging.basicConfig(
    encoding='utf-8',
    level=logging.DEBUG
)

def motor_talk(motor):
    # pitch is only a function of velocity, not of direction?
    # set to wheel mode, 
    motor.disable()
    time.sleep
    motor.set_operating_mode(1)
    motor.set_velocity_limit(2001)
    time.sleep(1)
    motor.enable()
    pitch_list = [800,1200,2000]
    pitch_list = [100,200,400]
    len_list = [0.2,0.4,0.8]
    len_list = [0.1,0.2]
    move_sign = 1
    try:
        for _ in range(100):
            cur_len,cur_pitch = random.choice(len_list),random.choice(pitch_list)
            motor.set_goal_velocity(move_sign*cur_pitch)
            move_sign *= -1
            time.sleep(cur_len)
    except:
        pass
    motor.set_goal_velocity(0)
    
def main():
    # Create the gadgets
    dxl_motor = r0b0.gadgets.from_config(
        os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),'../config/gadgets/dxl_motor.yaml')))
    dxl_motor.disable()
    # for motor in dxl_motor.dxl_dict.values():
    #     motor.disable()
    
    dxl_motor.POLL_MOVEMENT = True
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