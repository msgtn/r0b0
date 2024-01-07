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
# print(uuid.uuid4())
import r0b0
from r0b0.gadgets import Gadget
from r0b0.gadgets.time_controller import *
from flask_socketio import SocketIO, Namespace
from flask import jsonify
import socketio
# def listen():
#     while True:
#         # bg_emit()
#         eventlet.sleep(5)

# eventlet.spawn(listen)
# import logging
# logging.basicConfig(
#     encoding='utf-8',
#     level=logging.DEBUG
# )

# TC = TimeController(
#     config={'type':'TimeController','name':'tc'}
# )

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
    # breakpoint()

# def test_emit(rig):
    # tc = TimeController()
    # rig.add_gadget(tc)
    # tc.emit(
    #     event='test',
    #     data={}
    # )
    # pytest.mark

@pytest.mark.parametrize('mode',['idle','stopwatch','timer'])
def test_set_mode(rig, mode):
    # tc_name = uuid.uuid4()
    # TC.name = tc_name = str(uuid.uuid4())
    # assert TC.name == tc_name
    # assert TC.namespace == f'/{tc_name}'
    # breakpoint()
    # breakpoint()
    # rig.server.eio.async_mode = 'threading'
    if not rig.power:
        rig.add_gadget(TC)
        rig.add_gadget(test_gadget)
        # rig.start()
        # breakpoint()
        rig.power_on()
        # rig.start_background_task(
        #     rig.power_on
        # )

    emit_kwargs = {
        'event':'set_mode',
        'mode':mode
    }
    emit_data = Message(**emit_kwargs)
    emit_data_dict={
            'event':emit_data.event,
            # 'msg':pickle.dumps(emit_data),
            # 'msg':pickle.dumps(emit_data).decode("utf-8", errors="ignore"),
            'msg':pickle.dumps(emit_data).hex(),
            # 'msg':str(pickle.dumps(emit_data)),
        }
    
 
    # rig.on_forward(
    # rig.emit(
    #     event=emit_data.event,
    #     # to=None,
    #     data=emit_data_dict,
    #     # include_self=False,
    #     namespace=TC.namespace
    # )
    # time.sleep(1)
    # test_client = rig.test_client(
    #     app=rig.app
    # )
    # TC.name = ""
    # test_client.connect()
    # sio = socketio.SimpleClient()
    # sio.connect(
    #     url
    # )
    # TC_ns = Namespace(namespace=TC.namespace)
    # breakpoint()
    # test_gadget.emit('forward',data={
    #         'event':'set_mode',
    #         'mode':'stopwatch',
    #         'namespace':'/time_controller'
    #         },
    #     )
    import requests
    # print(os.path.join(CONFIG_DIR, '../examples/csr.pem'))
    CSR_PEM = os.path.abspath(
        os.path.join(CONFIG_DIR, '../../examples/csr.pem')
    )
    # print(CSR_PEM)
    # breakpoint()
    res = requests.post(
        f'https://localhost:8080/forward{TC.namespace}/set_mode',
        # f'http://localhost:8080/forward{TC.namespace}/set_mode',
        # data=emit_kwargs,
        
        data=json.dumps(emit_data_dict),
        # data=emit_data_dict,

        # data=json.dumps(emit_kwargs),
        # data=pickle.dumps(emit_data),
        # data=jsonify({
        #     'event':'set_mode',
        #     # 'msg':emit_data_dict,
        #     'mode':mode,
        #     }),
        # data=emit_data_dict,
        # data=json.dumps({'data':emit_data}),
        headers={'Content-Type':'application/json'},
        # verify=os.path.join(CONFIG_DIR, '../../../examples/csr.pem')
        # verify=CSR_PEM,
        verify=False
        )
    print(res)
    # breakpoint()
    time.sleep(1)

    # TC.set_mode_event()
    assert TC.mode == getattr(TimeMode, mode.upper())