import os
import random
import logging

logging.basicConfig(
    encoding="utf-8",
    # level=logging.DEBUG,
    # level=logging.WARNING,
    level=logging.INFO,
)
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig
from r0b0.cables.blsm import (
    Motion2MotorCable,
    Microphone2PromptCable,
    Text2PromptCable,
    Key2MicCable,
    Serial2PoseCable,
    Text2PoseCable,
    Response2PoseCable,
)
from r0b0.cables.cable import Wav2MotorCable, Ser2MicCable, Response2ListenCable



CONFIG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../config/gadgets/")
)


def main():
    # Start the server
    # print(LOCALHOST)
    with open(
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../ngrok_public_url.txt")
        ),
        "r",
    ) as _file:
        socket_addr = _file.readlines()[0].strip()
    # print(LOCALHOST)

    rig = Rig(
        # Default: https://localhost:8080
        # hostname='0.0.0.0',
        hostname=LOCALHOST,
        port=SERVER_PORT,
        # Point to wherever you created the OpenSSL keys
        certfile=os.path.join(os.path.dirname(__file__), "csr.pem"),
        keyfile=os.path.join(os.path.dirname(__file__), "key.pem"),
        pages_folder=os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../pages/blsm/")
        ),
        socket_addr=socket_addr,
    )

    # Create the gadgets
    blsm_dxl = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "blsm_dxl.yaml"))
    blsm_phone = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "blsm_phone.yaml"))
    microphone = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "usb_audio.yaml"))
    lm = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "lm.yaml"))
    ser = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "serial.yaml"))
    # ser.listen()

    motion2motor_cable = Motion2MotorCable()
    mic2prompt_cable = Microphone2PromptCable()
    wav2motor_cable = Wav2MotorCable()
    ser2mic_cable = Ser2MicCable()
    res2listen_cable = Response2ListenCable()

    listen = lambda: ser.listen()
    lis = listen
    blsm_txts = [
        "blsm",
        "BLSM",
        "blossom",
        "Blossom",
        "b l o s s o m",
        "B l o s s o m",
        "B L O S S O M"
    ]
    # write_blsm = lambda: ser.write(random.choice(blsm_txts))
    def write_blsm():
        txt = random.choice(blsm_txts)
        padding = (15-len(txt))//2

        empty_line = " "*15
        txt = empty_line*3 + " "*padding + txt

        ser.write(txt)
    wb = write_blsm
    def all_stop():
        [rig.stop(x) for x in list(rig.tapes.keys())]
    stop = all_stop

    if "blsm_dxl" in locals():
        rig.add_cable(
            cable=motion2motor_cable,
            rx_gadget=blsm_dxl,
            tx_gadget=blsm_phone,
        )
        rig.add_cable(
            cable=wav2motor_cable,
            rx_gadget=blsm_dxl,
            tx_gadget=lm,
        )
        rig.add_cable(
            cable=Response2PoseCable(),
            rx_gadget=blsm_dxl,
            tx_gadget=lm,
        )
        rig.add_cable(
            cable=Text2PoseCable(),
            rx_gadget=blsm_dxl,
            tx_gadget=microphone,
        )


    rig.add_cable(
        cable=mic2prompt_cable,
        rx_gadget=lm,
        tx_gadget=microphone,
    )
    rig.add_cable(
        cable=Text2PromptCable(),
        tx_gadget=blsm_phone,
        rx_gadget=lm,
    )
    if "ser" in locals():
        rig.add_cable(
            cable=res2listen_cable,
            rx_gadget=ser,
            tx_gadget=lm,
        )
        rig.add_cable(
            cable=ser2mic_cable,
            rx_gadget=microphone,
            tx_gadget=ser)
        rig.add_cable(
            cable=Serial2PoseCable(),
            rx_gadget=blsm_dxl,
            tx_gadget=ser,
        )

    test_emit_dict = {
        "event": "position",
        "data": {
            "event": "position",
            "msg": b"\x80\x04\x95y\x00\x00\x00\x00\x00\x00\x00\x8c\x13r0b0.gadgets.gadget\x94\x8c\x07Message\x94\x93\x94)\x81\x94}\x94(\x8c\x05event\x94\x8c\x08position\x94\x8c\x05value\x94]\x94(M/\x03M\x93\x06M\xd8\x06M\xd4\x07e\x8c\x08motor_id\x94]\x94(K\x01K\x02K\x03K\x04e\x8c\x08absolute\x94\x88ub.",
        },
        "to": None,
        "include_self": False,
        "namespace": "/blsm_dxl",
    }

    # Power on the rig
    rig.power_on()
    try:
        # Serve indefinitely
        breakpoint()
    except KeyboardInterrupt:
        # Power off the rig
        rig.power_off()
        exit()


if __name__ == "__main__":
    main()
