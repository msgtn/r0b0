import numpy as np

try:
    from r0b0.kinematics.blsm import device_motion2dxl_motor
except:
    pass
from r0b0.utils.loaders import decode_msg, encode_msg
import pickle
import logging

# def msg_func(func):
#     return


def msg_func(data=None, func=None, input_event=None, output_event=None):
    if data is None:
        return {"event": input_event}
    return {"event": output_event, **func(data)}


# @load_pickle
def cc2motor(data=None):
    if data is None:
        return {"event": "midi_cc"}
    # data = pickle.loads(data['msg'])
    # print(data)
    return {
        "event": "position",
        "value": (data.value * 4096) // 127,
        "motor_id": (data.control),
    }


# @load_pickle
def cc2ard(data=None):
    if data is None:
        return {"event": "midi_cc"}
    # data = pickle.loads(data['msg'])
    # print(data)
    return {
        "event": "position",
        "value": int(np.interp(data.value, [0, 127], [10, 160])),
        "motor_id": 10,
    }


@decode_msg
def note2motor(data=None):
    """
    C4 = note value 60
    """
    if data is None:
        return {"event": "midi_on"}
    # data = pickle.loads(data['msg'])
    return {
        "event": "position",
        "value": int(np.interp(data.note, [53, 77], [0, 4096])),
        "motor_id": (data.channel + 1),
    }


# @load_pickle
# def motion2motor(data=None):
#     if data is None: return {'event':'device_motion'}
#     # logging.debug(f'motion2motor {data}')
#     return {
#         'event':'position',        # 'value': # the function that gives
#         'value':device_motion2dxl_motor(data),
#         'motor_id':[1,2,3,4]
#     }


def motion2ardmotor(data=None):
    if data is None:
        return {"event": "device_motion"}
    logging.debug(f"motion2ardmotor {data}")
    return {
        "event": "position",  # 'value': # the function that gives
        "value": [int(np.interp(data["x"], [0, 1.5], [20, 160]))],
        "motor_id": [9],
    }


def motion2midi(data=None):
    if data is None:
        return {"event": "device_motion"}
    note = int(np.interp(np.rad2deg(data["x"]), [0, 180], [40, 90]))
    return {
        "event": "midi",
        "type": "note_on",
        "note": note,
        "velocity": 100,
        "channel": 7,
    }


def button2cam(data=None):
    if data is None:
        return {"event": "pi_button"}
    return {
        "event": data["button"],
    }


def joy2midi(data=None):
    if data is None:
        return {"event": "joybutton"}
    event_dict = {
        "joybuttondown": "note_on",
        "joybuttonup": "note_off",
    }
    # print('joy2midi')
    # data = pickle.loads(data['msg'])
    # logging.debug(data)
    return {
        "event": "midi",
        "type": data["event_type"],
        "note": data["button"] + 40,
        "velocity": 100,
        "channel": 4,
    }


def joy2dxlmotor(data=None):
    if data is None:
        return {"event": "joyaxismotion"}
    # if data['axis']!=3: return
    pos_val = data["value"]
    # pos_val /= np.abs(pos_val)
    # pos_val = np.sin(pos_val*np.pi/2)
    logging.debug(pos_val)
    return {
        "event": "position",
        "value": int(np.interp(pos_val, [-1, 1], [0, 4096])),
        # 'motor_id':(data['axis']+1)
        # 'motor_id':(data['axis']+1)
        "motor_id": data["axis"],
    }


def joy2ardmotor(data=None):
    if data is None:
        return {"event": "joyaxismotion"}
    # if data['axis']!=3: return
    pos_val = data["value"]
    # pos_val /= np.abs(pos_val)
    # pos_val = np.sin(pos_val*np.pi/2)
    logging.debug(pos_val)
    return {
        "event": "position",
        "value": int(np.interp(pos_val, [-1, 1], [10, 170])),
        # 'motor_id':(data['axis']+1)
        # 'motor_id':(data['axis']+1)
        "motor_id": 10 if data["axis"] == 3 else 8,
    }


# def dpad2shutter(data=None):
#     if data is None: return {'event':''}


def joy2mouse_move(data=None):
    if data is None:
        return {"event": "joyaxismotion"}
    axis, value = data["axis"], data["value"]
    # logging.debug(data)
    data.update({"event": "mouse_move"})
    return data


def joy2mouse_button(data=None):
    if data is None:
        # return {'event':'joybuttondown'}
        return {"event": "joybutton"}
    logging.debug(data)
    BUTTON2MOUSE = {
        0: "left",
        1: "right",
        2: "middle",
    }
    BUTTON2MOUSE.setdefault("left")

    mouse_func = {"button_down": "press", "button_up": "release"}[data["button_press"]]

    data.update(
        {
            "event": "mouse_button",
            "mouse_func": mouse_func,
            "kwargs": {
                "button": BUTTON2MOUSE.get(data["button"], "left"),
            },
        }
    )
    return data


def key2cam(data=None):
    if data is None:
        return {"event": "keydown"}

    if data["unicode"] == "c":
        return {"event": "read", "save": True}


def key2mic(data=None):
    if data is None:
        return {"event": "keydown"}
    if data["unicode"] == "m":
        return {
            "event": "listen",
        }


def text2prompt(data=None):
    if data is None:
        return {"event": "text"}
    logging.warning(data)
    return {"event": "prompt", "prompt": data["text"]}


def text2yes_no_prompt(data=None):
    if data is None:
        return {"event": "text"}
    logging.warning(data)
    data["text"] += "? Respond with either yes or no."
    logging.warning(data["text"])
    return {"event": "prompt", "prompt": data["text"]}
