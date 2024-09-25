import os
import logging

logging.basicConfig(
    encoding="utf-8",
    # level=logging.DEBUG,
    level=logging.WARNING,
)
import r0b0
from r0b0.config import LOCALHOST, SERVER_PORT
from r0b0.rigs import Rig


CONFIG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../config/gadgets/")
)


def main():
    # Start the server

    rig = Rig(
        hostname=LOCALHOST,
        port=SERVER_PORT,
    )

    # Create the gadgets
    mpi_camera = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "mpi_camera.yaml"))
    mpi_buttons = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, "mpi_buttons.yaml"))
    mpi_cable = MPiCable()
    rig.add_cable(
        cable=mpi_cable,
        rx_gadget=mpi_camera,
        tx_gadget=mpi_buttons,
    )

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
