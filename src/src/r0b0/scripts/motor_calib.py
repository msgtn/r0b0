import logging

logging.basicConfig()

from dynamixel_python import DynamixelManager, DynamixelMotor, ReadError

MOTOR_ID_LIMIT = 20
# for XL330 motors
MOTOR_MODEL, USB_PORT, BAUD_RATE = "xl330-m288", "/dev/tty.usbserial-FT1SF1UM", 57600
# for XL320 motors
MOTOR_MODEL, USB_PORT, BAUD_RATE = "xl320", "/dev/tty.usbmodem212401", 1e6
BAUD_DICT = {57600: 1, 115200: 2, 1000000: 3, 9600: 0}

dxl_mgr = DynamixelManager(
    usb_port=USB_PORT,
    baud_rate=BAUD_RATE,
)


def add_motors(motor_id_limit=MOTOR_ID_LIMIT) -> None:
    for motor_id in range(motor_id_limit):
        dxl_mgr.add_dynamixel(
            dxl_id=motor_id, dxl_model=MOTOR_MODEL, dxl_name=str(motor_id)
        )
    dxl_mgr.init()


def get_connected_motors() -> list[int]:
    return [
        int(motor_id) for motor_id, motor in dxl_mgr.dxl_dict.items() if motor.ping()
    ]


# alias
gcm = get_connected_motors


def set_ids(id_dict) -> None:
    for old_id, new_id in id_dict.items():
        dxl_mgr.dxl_dict[str(old_id)].set_id(new_id)


def set_param(param, id_dict) -> None:
    for old_id, new_id in id_dict.items():
        getattr(dxl_mgr.dxl_dict[str(old_id)], f"set_{param}")(new_id)


if __name__ == "__main__":

    add_motors(MOTOR_ID_LIMIT)
    dxl_mgr.enable_all()
    # set_ids({
    #     1:3
    # })

    print(f"Found motors with ids {get_connected_motors()}")
    # m1 = dxl_mgr.dxl_dict['1']
    globals().update(
        {f"m{m_id}": dxl_mgr.dxl_dict[str(m_id)] for m_id in range(MOTOR_ID_LIMIT)}
    )
    breakpoint()

    """
    Usage
    gcm() # check which motors
    set_ids({1:5}) # set motor 1 to motor 5
    gcm() # should print 5
    dxl_mgr.close() # save changes
    # ctrl+D to exit
    """
