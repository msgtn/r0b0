# import ssl
# import eventlet
# eventlet.monkey_patch(
#     # all=True,
#     os=False,
#     select=False,
#     socket=True,
#     thread=Truee,
#     time=False
# )

import pytest
import uuid
import os, time, pickle, json
import requests
# print(uuid.uuid4())
import r0b0
from r0b0.gadgets import Gadget
from r0b0.gadgets.dxl_robot import *
from r0b0.gadgets.time_controller import *
from flask_socketio import SocketIO, Namespace
from flask import jsonify
import socketio
from timeit import default_timer

CONFIG_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),'../../../config/gadgets/'))
TC = r0b0.gadgets.from_config(os.path.join(CONFIG_DIR, 'time_controller.yaml'))
test_gadget = Gadget()
test_gadget.name = ''

# def test_init():
#     TimeController()

def test_enums():
    time_enum = TimeMode


@pytest.mark.parametrize('mode',['idle','stopwatch','timer'])
def test_set_mode(rig, mode):
    if not rig.power:
        rig.add_gadget(TC)
        rig.power_on()

    emit_kwargs = {
        'event':'set_mode',
        'mode':mode
    }

    rig.manual_emit(
        event='set_mode',
        data={
            'event':'set_mode',
            'msg':TC.Message(**emit_kwargs)
        },
        namespace=TC.namespace
    )
    # Wait for the gadget to update
    time.sleep(1)

    assert TC.mode == getattr(TimeMode, mode.upper())


@pytest.mark.parametrize('mode',['idle','stopwatch','timer'])
def test_get_mode(rig, mode):
    # TC.mode = mode
    # assert TC.get_mode()
    pass


@pytest.mark.parametrize('mode',['stopwatch','timer'])
def test_mode(mode):
    # initialize
    # set the motor to idle mode
    TC.mode = TimeMode.IDLE
    assert TC.mode == TimeMode.IDLE

    # timer should be emitting position events to the motor
    # that are moving counter-clockwise if in timer mode
    # and clockwise if in stopwatch mode
    # how can we check that the server is receiving emits?
    # try creating a specific event handler
    time_position_handler_called = False
    last_position = np.inf if mode=='stopwatch' else -np.inf
    def time_position_handler(data):
        if not time_position_handler_called:
            time_position_handler_called = True
        msg = data['msg']
        # make sure the event is recent (converted to seconds(?))
        assert ( (msg.time_sent - time.time()) / 1e3 ) < 1.

        if not (TC.mode == TimeMode.IDLE):
            # check that the positions are in the right direction
            if mode=='stopwatch':
                assert msg.position < last_position
            else:
                assert msg.position > last_position
        last_position = msg.position
        pass
    rig.on_event(
        'time_position',
        handler=time_position_handler,
        namespace=TC.namespace,
    )
    # also handle a reset event
    reset_called = False
    def reset_handler(data):
        if not reset_called:
            reset_called = True
    rig.on_event(
        'reset',
        handler=reset_handler, 
        namespace=TC.namespace
    )

    # simulate motor movement events
    # in the clockwise direction (negative velocities?)
    # for a few seconds
    # then stop
    time_start = default_timer()
    while (default_timer()-start) < 2.0 :
        # emit what comes *out* of the motor event
        # motor_velocity -> Cable -> time_controller_event
        # maybe just passthrough motor_velocity
        # on the time controller end, must keep track of the last 
        # motor_velocity input event, to give buffer for when user stopped moving
        rig.manual_emit(
            event='motor_velocity',
            data={
                'event':'motor_velocity',
                'msg':DynamixelRobot.Message(**{
                    'event':'motor_velocity',
                    'value':1 if mode=='stopwatch' else -1,
                })
            }
        )
        # check that the motor is in timer mode
        assert TC.mode == getattr(TimeMode, mode.upper())
    
    # ensure that the time controller pauses before starting 
    # TODO - make this pause time a config param
    time_motor_stop = default_timer()
    while (default_timer() - time_motor_stop) < 2.0 : 
        assert not time_position_handler_called
    
    # the time controller should be emitting 'time_position' events
    # and the handler will be called
    time.sleep(2)
    assert time_position_handler_called
    pass

    # simulate user moving the motor
    # this might need a debounce
    rig.manual_emit(
        event='motor_velocity',
        data={
            'event':'motor_velocity',
            'msg':DynamixelRobot.Message(**{
                'event':'motor_velocity',
                'value':1 if mode=='stopwatch' else -1,
            })
        }
    )
    time.sleep(1)

    # check that the timer has switched to idle mode
    assert TC.mode == TimeMode.IDLE
    # check that the time controller is moving the 
    # motor back to its reset position
    # maybe this is a separate reset function?
    assert reset_called


def test_cable_that_sends_multiple_events():
    # might need to send multiple events in a cable
    # if we separate this logic from the gadgets
    # this may be a different case though
    # since this is a bespoke gadget
    # it may be useful anyways to keep fancy logic in a cable
    # without having to modify gadgets
    # maybe time controller should have subclassed Rig instead of Gadget

    # time_position -> Cable -> [enable_motor, motor_position, disable_motor]
    pass
