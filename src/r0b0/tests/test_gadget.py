from r0b0.gadgets import Gadget
from r0b0.rigs import Rig
from r0b0.config import LOCALHOST, SERVER_PORT
import pytest

config = {"type": "Gadget"}

# rig = Rig(
#     hostname=LOCALHOST,
#     port=SERVER_PORT,
# )


@pytest.fixture(autouse=True)
def boilerplate():
    gadget = Gadget(config)
    yield


@pytest.fixture
def rig():
    return Rig(
        hostname=LOCALHOST,
        port=SERVER_PORT,
    )


@pytest.fixture
def gadget():
    return Gadget(config)


def test_gadget_init(gadget):
    assert Gadget(config) is not None

    assert Gadget() is not None


def test_add_to_rig(gadget, rig):
    # gadget = Gadget(config)
    rig.add_gadget(gadget)
    assert gadget.name in rig.gadgets
    # rig.power_on()
    # rig.power_off()
    # return gadget


# def test_send_to_rig(gadget):
#     gadget = test_connect_to_rig
