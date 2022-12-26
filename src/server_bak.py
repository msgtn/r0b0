
"""
A Flask webserver for handling requests to the robot
"""
from __future__ import print_function

import os
# from . import kinematics as k
# from src import kinematics as k
from flask import Flask, render_template, request, jsonify, send_file, make_response
from flask_socketio import SocketIO
import json
import numpy as n
from collections import OrderedDict
import socket
import requests
from flask_cors import CORS
import time
import io
from engineio.payload import Payload
Payload.max_decode_packets = 50

class Server(object):
    """
    A "singleton" object for housing server state. Includes methods for updating server functions from outside sources.
    """

    def __init__(self, port="8000"):
        # self.master_robot = None
        # self.robots = []
        # self.handle_input = lambda: None
        # self.record = lambda: None
        # self.stop_record = lambda: None
        # self.store_gesture = lambda: None

        # # playback params
        # self.speed = 1.0
        # self.amp = 1.0
        # self.post = 1.0

        # # init yaw (for resetting w/ phone controller)
        # self.yaw = 0
        # # init dict of current motor positions
        # self.motor_pos = {}

        self.port = port
        # self.save_latency = False
        # self.latency = []
        # self.recording = False

    # def set_funcs(self, master_robot, robots, handle_input, record, stop_record, store_gesture):
    #     """
    #     updates server functions
    #     """
    #     self.master_robot = master_robot
    #     self.robots = robots
    #     self.handle_input = handle_input
    #     self.record = record
    #     self.stop_record = stop_record
    #     self.store_gesture = store_gesture

    def start_server(self, host, port):
        """
        initialize the flask server
        """
        self.port = port
        # app.run(host=host, port=port, threaded=True)
        socketio.run(app,
            host=host, port=port, 
            keyfile='key.pem', certfile='cert.pem',
            debug=True
        )


app = Flask(__name__) 
app.config['SECRET_KEY'] = 'secret!'
CORS(app)
socketio = SocketIO(app,
    cors_allowed_origins='*',
   async_mode='eventlet'
)
server = Server()
leg_server = Server()
# cur_yaw = 0

# # paths relative to start.py, should make relative to this file in the future
# SEQUENCES_DIR = "./src/sequences/"
# REACTIONS_DIR = "./src/reactions/"


# # @app.before_request
# # def add_cors_headers_before():
# #     response = make_response()
# #     response.headers.add('Access-Control-Allow-Origin', 'https://blossomapp.ngrok.io')
# #     return response
#     # return add_cors_headers(response, origins='https://blossomapp.ngrok.io')
# @app.after_request
# def add_cors_headers_after(response):
#     return add_cors_headers(response)


# def add_cors_headers(response):
#     """
#     adds cors headers
#     """
#     response.headers.add('Access-Control-Allow-Origin', '*')
#     response.headers.add('Access-Control-Allow-Credentials', 'true')
#     response.headers.add('Access-Control-Allow-Headers', 'Content-Type')
#     response.headers.add('Access-Control-Allow-Headers', 'Cache-Control')
#     response.headers.add(
#         'Access-Control-Allow-Headers', 'X-Requested-With')
#     response.headers.add('Access-Control-Allow-Headers', 'Authorization')
#     response.headers.add('Access-Control-Allow-Methods',
#                          'GET, POST, OPTIONS, PUT, DELETE')
#     return response


# @app.route('/r')
# def handle_reload():
#     """
#     reload list of sequences
#     """
#     server.handle_input(server.master_robot, 'r')
#     return "reloaded"

# @app.route('/r/<gesture>')
# def handle_reload_gesture(gesture):
#     """
#     reload list of sequences
#     """
#     server.handle_input(server.master_robot, 'r', [gesture])
#     return "reloaded"

# @socketio.on('gesture')
# def handle_gesture_socket(gesture):
#     play_sequence(gesture,return_msg=False);

# @socketio.on('deleteGesture')
# def handle_delete_gesture_socket(gesture):
#     robot_name = server.robots[0].name
#     gesture_fn = './src/sequences/{}/{}_sequence.json'.format(robot_name,gesture)
#     print(os.getcwd(), gesture_fn)
#     if os.path.exists(gesture_fn):
#         os.remove(gesture_fn)
#     server.robots[0].seq_list.pop(gesture)
#     handle_reload()

# @app.route('/s/<gesture>')
# def handle_sequence(gesture):
#     play_sequence(gesture,return_msg=True)

# def play_sequence(gesture,return_msg=False):
#     """
#     plays a sequence
#     """
#     speed, amp, post = request.args.get('speed'), request.args.get('amp'), request.args.get('post')

#     if speed==None: speed = 1
#     if amp==None: amp = 1
#     if post==None: post = 1

#     server.speed = float(speed)
#     server.amp = float(amp)
#     server.post = float(post)

#     for bot in server.robots:
#         bot.speed = server.speed
#         bot.amp = server.amp
#         bot.post = server.post

#     server.handle_input(server.master_robot, 's', [gesture])
#     if return_msg:
#         return "200 OK"
#         return gesture, "fired"


# @app.route('/s/<gesture>/<idle>')
# def handle_sequence_idle(gesture, idle):
#     """
#     plays a sequence, repeating indefinitely
#     """
#     try:
#         server.handle_input(server.master_robot, 's', [gesture + '/' + idle])
#     except KeyError as e:
#         print("Unknown sequence", e)
#         pass
#     return gesture, "fired"

# height = 50

# @app.route('/position', methods=['POST'])
# def handle_set_position_http():
#     j = request.get_json()
#     return set_position(j)

# @socketio.on('connect')
# def handle_connect():
#     print("Socket connected")

# @socketio.on('position')
# def handle_set_position_socket(json):
#     server.save_latency = True
#     set_position(json)

# tracking = False
# @socketio.on('tracking')
# def handle_tracking(tracking_on):
#     global tracking
#     tracking = tracking_on
#     if not tracking: reset_sensors()
#     print(f'Tracking {tracking}')

# @socketio.on('track')
# def handle_track_socket(tracking_error):
#     global tracking
#     if not tracking: return
#     tracking_error = np.array(tracking_error)
#     # if tracking_error[0]==0 and tracking_error[1]==0:
#     if not any(tracking_error):
#         reset_sensors()
#         return
#     # print(f'track {tracking_error}')
#     # get the current position of the base motor
#     bot = server.robots[0]
#     base_pos = bot.get_motor_pos()['base']
#     neck_pos = bot.get_motor_pos()['tower_1']
#     # calculate control signal
#     # TODO - PID tuning
#     # u_lim = 0.5
#     # u = u_lim/tracking_error

#     gain = np.array([30,1])
    
#     u = gain*np.sign(tracking_error)*np.cos(tracking_error*np.pi)

#     # u = -gain/np.sin(tracking_error*np.pi)
#     # u = np.array([np.max([np.min([1,_u]),-1]) for _u in u])
#     # print(u)
    
#     # calculate goal position
#     # base_pos += 40*u
#     base_pos += u[0]
#     base_pos = np.min([np.max([base_pos,-130]),130])
#     neck_pos -= u[1]
#     neck_pos =np.min([np.max([neck_pos,-30]),90])
#     # print(base_pos)
#     bot.goto_position(
#         {
#             'base':base_pos,
#             'tower_1':neck_pos,
#         },
#         delay=0.001,
#     )
#     print(base_pos, neck_pos)

# def set_position(raw_data):
#     """
#     moves the robot to the positon specified by the request data
#     """


#     motor_pos = imu_to_motor_pos(raw_data)

#     # adjust the duration (numeric input to goto_position)
#     # higher (0.3) = slow+smooth, low (0.1) = jittery+fast
#     for bot in server.robots:
#         del_pos = np.abs(np.array(list(motor_pos.values())) - np.array(list(bot.believed_motor_pos.values())))

#         dof_ctrl = []
#         if raw_data['headCtrl']:
#             dof_ctrl += ['tower_1','tower_2','tower_3','base']
#         if raw_data['armsCtrl']:
#             dof_ctrl += ['ears','left_arm','right_arm','tail']
#             print(motor_pos)


#         # limit movement to avoid spinning around
#         # for i,d_pos in enumerate(del_pos):
#         #     if d_pos > 100:
#         bot.goto_position(
#             {dof:motor_pos[dof] for dof in dof_ctrl},
#             delay=0.1,
#             # delay=0.2,
#             # delay=0.01,
#             # wait=True,
#             wait=False,
#         )
#         bot.believed_motor_pos = motor_pos

#     server.motor_pos = motor_pos
#     return "200 OK"
#     # return str(server.yaw)

# def imu_to_motor_pos(raw_data, sensitivity=1.0):
#     # get data as string
#     # data is provided as rotations wrt y,-x,-z (w/ screen facing up)
#     # order is pitch/y, (-)roll/(-)x, -yaw(-z), height, ears, acc_x, acc_y, acc_z
#     # split into measurements
    
#     mirror = raw_data['mirror']
#     height_ctrl = raw_data['heightCtrl']
#     arms_ctrl = raw_data['armsCtrl']
#     head_ctrl = raw_data['headCtrl']
#     global server
#     pos = k.get_motor_pos(
#         [raw_data['x'], raw_data['y'], raw_data['z'], raw_data['h'], server.yaw],
#         portrait=raw_data['portrait'],
#         sensitivity=sensitivity,
#         )

#     # try:
#     #     # difference between now and when data was sent
#     #     diff_time = np.abs(time.time()*1000-raw_data['time'])
#     #     if server.save_latency and server.recording:
#     #         server.latency.append(diff_time)
#     #     else:
#     #         server.latency = [diff_time]
#     #     # print(diff_time)
#     #     # check if lagging, print if slow
#     #     if diff_time > 1500:
#     #         # if very slow, immediately return to attenuate lag
#     #         if diff_time > 2000:
#     #             print(diff_time)
#     #             return "200 OK"
#     #         pass
#     # except:
#     #     pass

#     global height
#     if height_ctrl:
#         height = np.interp(-raw_data['x'], [0.5,1.5,],[0,100])
#         raw_data['h'] = height
#     imu = get_imu_data(raw_data)

#     if height_ctrl:
#         for i in [0,1,2]:
#             imu[i] = 0
#         # return "200 OK"
#     # else:
#     #     imu[3] = height

#     if server.yaw == -999:
#         server.yaw = imu[2]
#     # get base motor positions, accounting for stored yaw reset position
#     global cur_yaw
#     cur_yaw = imu[2]
#     adj_yaw = imu[2]-server.yaw

#     # get ear motor position
#     ears_pos = k.get_ears_pos(imu[4])
#     # save orientation
#     ori = [-imu[2], imu[0], -imu[1]]

#     # filter out readings below certain threshold
#     accel = [imu[4], imu[5], imu[6]]

#     k.integrate_accel(ori, accel)

#     # command positions
#     motor_pos = {
#         # 'tower_1': pos[0],
#         'tower_1': 0.8*(pos[0]-80),
#         'tower_2': pos[1],
#         'tower_3': pos[2],
#         'base': pos[3],
#         'ears': ears_pos,
#         'left_arm':raw_data['left_arm'],
#         'right_arm':raw_data['right_arm'],
#         'tail':raw_data['tail'],
#     }

#     # handle mirroring mode (swap roll for towers 2 and 3, flip yaw)
#     if (mirror):
#         motor_pos['tower_2'] = pos[2]
#         motor_pos['tower_3'] = pos[1]
#         motor_pos['base'] = -pos[3]

#     # prevent quick turning around
#     if 'base' in server.motor_pos:
#         last_yaw = server.motor_pos['base']
#         if(np.abs(last_yaw - motor_pos['base']) > 100):
#             motor_pos['base'] = last_yaw
#     # print(motor_pos)

#     return motor_pos



# def get_imu_data(raw_data):
#     """
#     return the current IMU values of the robot
#     """
#     if raw_data['landscape']:
#         data_order = ['y', 'x', 'z', 'h', 'ears', 'ax', 'ay', 'az']
#         imu = [raw_data[x] for x in data_order]

#         imu[0] = -imu[0]-np.pi/2
#         imu[0] = np.max([np.min([imu[0],1.5]),-0.7])
#     else:
#         data_order = ['x', 'y', 'z', 'h', 'ears', 'ax', 'ay', 'az']
#         imu = [raw_data[x] for x in data_order]

#     return imu


# @app.route('/sequences')
# def get_sequences():
    
#     # seqs = server.master_robot.get_time_sequences()
#     seqs = server.master_robot.get_sequences()

#     return jsonify(seqs)


# # TODO: support multiple robots and move logic to start.py (in case we want to update from CLI)
# # @app.route('/sequences/<seq_id>', methods=['POST'])
# @app.route('/sequences/<seq_id>')
# def update_sequence(seq_id):
#     """
#     Updates a sequence's name. If the sequence was temporary before, make it a persistant sequence.
#     """
#     # load the data from the gesture generation form
#     data = json.loads(request.data)
#     if "name" not in data:
#         return "no name given", 400
#     # split
#     name, label = data["name"], data["label"]

#     # directory stuff
#     seq_dir = "%s%s/" % (SEQUENCES_DIR, server.master_robot.name)
#     tmp_dir = seq_dir + "tmp/"
#     # if should belong in subdirectory, make directory and truncate sequence name
#     if ('/' in name):
#         seq_dir += name[:name.rfind('/') + 1]
#         if not os.path.isdir(seq_dir):
#             os.makedirs(seq_dir)
#         name = name[name.find('/') + 1:]

#     # handle collisions with existing gesture names
#     new_name = name
#     name_ctr = 1
#     while ((new_name + '_sequence.json') in os.listdir(seq_dir)):
#         new_name = name + '_' + str(name_ctr)
#         name_ctr += 1
#     name = new_name

#     # temporary file
#     src_file = "%s_sequence.json" % seq_id
#     # actual file
#     dst_file = "%s_sequence.json" % name

#     # move from temporary to actual
#     for seq in os.listdir(tmp_dir):
#         if seq == src_file:
#             os.rename(tmp_dir + src_file, seq_dir + dst_file)
#             update_seq_file(seq_dir + dst_file, name, label)
#             return "200 OK"

#     # change name in actual
#     for seq in os.listdir(seq_dir):
#         if seq == src_file:
#             os.rename(seq_dir + src_file, seq_dir + dst_file)
#             update_seq_file(seq_dir + dst_file, name, label)
#             return "200 OK"
#     return "sequence not found", 404


# def update_seq_file(seq_path, name, label):
#     """
#     updates the content of a sequence file to the given args.
#     """
#     seq = json.load(open(seq_path))

#     seq["animation"] = name
#     seq["label"] = label
#     with open(seq_path, "w") as f:
#         json.dump(seq, f, indent=2)

#     for robot in server.robots:
#         robot.load_sequence(seq_path)
#     server.store_gesture(name, seq["frame_list"], label)

# @app.route('/videos')
# def get_videos():
#     """
#     return a list of videos we have sequences for
#     """
#     # init dict of all video ids
#     video_ids = OrderedDict()
#     app.config["JSON_SORT_KEYS"] = False

#     # get path to directory storing reactions
#     video_dir = REACTIONS_DIR
#     if not os.path.exists(video_dir):
#         os.makedirs(video_dir)

#     # save all videos
#     for vid in os.listdir(video_dir):
#         # check if json
#         if (vid[-5:] == '.json'):
#             # load file
#             data = json.load(open(video_dir + vid))
#             # catch if video was already added
#             video_id = data["videoId"]
#             video_ids.update({video_id: data["triggers"]})
#     return jsonify(video_ids)


# @app.route('/reset', methods=['POST'])
# def reset_sensors():
#     """
#     move blossom to the yaw = 0 position
#     """
#     # reset robot position
#     global server
#     server.master_robot.reset_position()
#     server.motor_pos = server.master_robot.reset_pos
#     # get current yaw reading and store it
#     # server.yaw = cur_yaw
#     return "200 OK"
#     return jsonify({"yaw":str(cur_yaw)})


# @app.route('/record/start', methods=['POST'])
# def handle_record_start():
#     server.recording = True
#     server.record(server.master_robot)
#     return "200 OK"


# @app.route('/record/stop/<seq_name>', methods=['POST'])
# def handle_record_stop(seq_name):
#     # use colons to delimit folders
#     server.recording = False
#     seq_name = seq_name.replace(':','/')
#     if server.save_latency and '/' in seq_name:
#         # if ('/' in seq_name):
#         seq_dir = seq_name[:seq_name.rfind('/')]
#             # seq_name = seq_name[seq_name.rfind('/'):]
#         # make subdir if doesnt exist
#         if not os.path.exists('./latency/'+seq_dir):
#             os.makedirs('./latency/'+seq_dir)
#         with open('./latency/{}.txt'.format(seq_name), 'w') as f:
#             f.write('\n'.join([str(s) for s in server.latency]))

#         server.latency = []
#     name = server.stop_record(server.master_robot,seq_name=seq_name)
#     return jsonify({"name": name})

# @app.route('/record/stop', methods=['POST'])
# def handle_record_stop_tmp():
#     name = server.stop_record(server.master_robot,seq_name=False)
#     return jsonify({"name": name})



# @app.route('/', defaults={'path': ''})
# @app.route('/<path:path>')
# def render_gui(path):
#     """
#     catch all that renders the react app
#     """
#     return render_template('index.html')


# # get the current ip address of the computer
# def get_ip_address():
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     try:
#         s.connect(("8.8.8.8", 80))
#         return s.getsockname()[0]
#     except:
#         return "localhost"
#         # return "No IP address!"
