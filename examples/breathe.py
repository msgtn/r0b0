import json

import numpy as np

arr = []
event = "position"
AMP = 1000
TAU = 100
for i in range(2*TAU):
    # TODO - better "shape", like actual breathing
    if i<TAU:
        # value = i*10
        value = AMP*np.sin(i/TAU*np.pi/2)
    else:
        # value = 1000-(i%100)*10
        value = AMP*(1-np.cos((i-2*TAU)/TAU*np.pi/2))
    
    data = {
        "event":event,
        "value":[value]*3,
        "motor_id":[1,2,3],
        "absolute":True,
        # "time":i*1e3,
        "time":i*0.5e3,
        # "time":i*0.3e3, 
        "rx_namespace":"/blsm_dxl"
    }
    arr.append({
        "event":"echo",
        "data":data,
    })

# TAPES_DIR = os.path.join(os.path.dirname(__file__), '..', 'tapes')
from r0b0.config import TAPES_DIR
import os
import json
wave = [a['data']['value'][0] for a in arr]
import matplotlib.pyplot as plt 
plt.plot(wave)
# plt.show()
# breakpoint()
with open(os.path.join(TAPES_DIR, "breathe_meditate.json"), 'w') as _file:
    # json.dumps(arr)
    _file.write(json.dumps(arr))


