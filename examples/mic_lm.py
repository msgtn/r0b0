import os
import logging

logging.basicConfig(
    encoding='utf-8',
    # level=logging.DEBUG,
    level=logging.WARNING,
)
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig
from r0b0.cables.blsm import Motion2MotorCable, Microphone2PromptCable, Key2MicCable
from r0b0.cables.cable import Wav2MotorCable, Ser2MicCable, Response2ListenCable


CONFIG_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),'../config/gadgets/'))

def main():
    # Start the server
    # print(LOCALHOST)
    with open(os.path.abspath(os.path.join(os.path.dirname(__file__), '../ngrok_public_url.txt')), 'r') as _file:
        socket_addr = _file.readlines()[0].strip()
    # print(LOCALHOST)
    
    rig = Rig(
        # Default: https://localhost:8080
        # hostname='0.0.0.0',
        hostname=LOCALHOST,
        port=SERVER_PORT,
        # Point to wherever you created the OpenSSL keys
        certfile=os.path.join(os.path.dirname(__file__), 'csr.pem'),
        keyfile=os.path.join(os.path.dirname(__file__), 'key.pem'),
        pages_folder=os.path.abspath(os.path.join(os.path.dirname(__file__), '../pages/blsm/')),
        socket_addr=socket_addr,
    )


    microphone = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "microphone.yaml"))
    ser = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "serial.yaml"))
    # microphone = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "xreal_mic.yaml"))
    lm = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "lm.yaml"))
    # pygame_keys = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "pygame_keys.yaml"))
    # motor = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "dxl_motor.yaml"))
    key2mic_cable = Key2MicCable()
    mic2prompt_cable = Microphone2PromptCable()
    wav2motor_cable = Wav2MotorCable()
    ser2mic_cable = Ser2MicCable()
    res2listen_cable = Response2ListenCable()

    # rig.add_cable(
    #     cable=key2mic_cable, 
    #     rx_gadget=microphone,
    #     tx_gadget=pygame_keys,
    # )
    rig.add_cable(
        cable=ser2mic_cable, 
        rx_gadget=microphone,
        tx_gadget=ser
    )
    rig.add_cable(
        cable=mic2prompt_cable,
        rx_gadget=lm,
        tx_gadget=microphone,
        )
    
    rig.add_cable(
        cable=res2listen_cable,
        rx_gadget=ser,
        tx_gadget=lm,
    )
    # rig.add_cable(
    #     cable=wav2motor_cable,
    #     rx_gadget=motor,
    #     tx_gadget=lm,
    # )

    test_emit_dict = {'event': 'position', 'data': {'event': 'position', 'msg': b'\x80\x04\x95y\x00\x00\x00\x00\x00\x00\x00\x8c\x13r0b0.gadgets.gadget\x94\x8c\x07Message\x94\x93\x94)\x81\x94}\x94(\x8c\x05event\x94\x8c\x08position\x94\x8c\x05value\x94]\x94(M/\x03M\x93\x06M\xd8\x06M\xd4\x07e\x8c\x08motor_id\x94]\x94(K\x01K\x02K\x03K\x04e\x8c\x08absolute\x94\x88ub.'}, 'to': None, 'include_self': False, 'namespace': '/blsm_dxl'}
    
    # Power on the rig
    rig.power_on()
    try:
        # Serve indefinitely 
        breakpoint()
    except KeyboardInterrupt:
        # Power off the rig
        rig.power_off()
        exit()


if __name__=="__main__":
    main()