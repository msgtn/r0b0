import logging
logging.basicConfig()

from dynamixel_python import \
    DynamixelManager, DynamixelMotor, ReadError
    
MOTOR_ID_LIMIT = 20
MOTOR_MODEL = 'xl330-m288'
BAUD_DICT = {
    57600:1,
    115200:2,
    1000000:3,
    9600:0
}

dxl_mgr = DynamixelManager(
    usb_port='/dev/tty.usbserial-FT1SF1UM',
    baud_rate=57600,
    # baud_rate=10e6,
)

def add_motors(motor_id_limit=MOTOR_ID_LIMIT):
    for motor_id in range(motor_id_limit):
        dxl_mgr.add_dynamixel(
            dxl_id=motor_id,
            dxl_model=MOTOR_MODEL,
            dxl_name=str(motor_id)               
        )
    dxl_mgr.init()
    
def get_connected_motors():
    return [int(motor_id) for motor_id,motor in dxl_mgr.dxl_dict.items() if motor.ping()]
gcm = get_connected_motors

def set_ids(id_dict):
    for old_id,new_id in id_dict.items():
        dxl_mgr.dxl_dict[str(old_id)].set_id(new_id)
        
def set_param(param,id_dict):
    for old_id,new_id in id_dict.items():
        getattr(dxl_mgr.dxl_dict[str(old_id)], f"set_{param}")(new_id)


if __name__=="__main__":
    
    add_motors(MOTOR_ID_LIMIT)
    
    set_ids({
        7:1,
    })
    
    print(f"Found motors with ids {get_connected_motors()}")
    
    breakpoint()
    
    
    '''
    Usage
    gcm() # check which motors
    set_ids({1:5}) # set motor 1 to motor 5
    gcm() # should print 5
    dxl_mgr.close() # save changes
    # ctrl+D to exit
    '''