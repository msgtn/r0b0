
import r0b0

def main():
    # Start the server
    r0b0.init()

    # Create the gadgets
    blsm_dxl = r0b0.gadget_from_config('blsm_dxl')
    blsm_phone = r0b0.gadget_from_config('blsm_phone')
    motion2motor_cable = Motion2MotorCable()

    r0b0.add_cable(
        tx_gadget=blsm_dxl,
        rx_gadget=blsm_phone,
        cable=motion2motor_cable,
        )
    

if __name__=="__main__":
    main()