#%%
from src.robot import *
import random
# %%   
robot = Robot(usb_port='/dev/tty.usbserial-FT1SF1UM',baud_rate=57600)
robot.from_config(
    {
        'motor_1':{'dxl_id':1,'dxl_model':'xl330-m288'},
        'motor_2':{'dxl_id':2,'dxl_model':'xl330-m288'},
        'motor_3':{'dxl_id':3,'dxl_model':'xl330-m288'},
        'motor_4':{'dxl_id':4,'dxl_model':'xl330-m288'}
    }
)
robot.power_up()
# %%
# robot.move_motor_id([1,4],[random.choice(range(4000))]*2)
# robot.move_motor_id(1,random.choice(range(4000)))
robot.move_motor_id(
    list(range(1,5)),
    # [random.choice(range(4000)) for _ in range(4)]
    [2048]*4
)
# %%
# robot.goto_position()
print(robot.motors)
# %%
rand_pos = random.choice(range(-150,150))
robot.goto_position(dict(
    motor_1=rand_pos
))
print(rand_pos)
# %%
# robot.motor_fn(1, 'get_present_velocity')
robot.motor_fn(1, 'get_profile_velocity')
# %%
import mido
mido.get_input_names()
note_range = [53, 76]
motor_range = [0,4000]
cc_range = [0,127]
with mido.open_input('OP-Z Bluetooth') as port:
    for msg in port:
        if msg.type=='note_on' and msg.channel<5:
            print(msg.note)
            motor_pos = int(np.interp(msg.note, note_range, motor_range))
            robot.move_motor_id(msg.channel+1, motor_pos)
        
        if msg.type=='control_change':
            print(msg)
            motor_pos = int(np.interp(msg.value, cc_range, motor_range))
            robot.move_motor_id(msg.control, motor_pos)
# %%
with mido.open_input('OP-Z Bluetooth') as port:
    for msg in port:
# %%
