"""
On computer, go to https://localhost:8080/blsm_broadcast
On mobile, go to https://r0b0.ngrok.io/blsm_controller
"""

import eventlet
import random
import json

eventlet.monkey_patch()

import os
import time
from threading import Thread
from typing import Optional

# Set to True to disable ROS2 imports for testing without ROS2
NO_ROS = False

if not NO_ROS:
    import rclpy
    from geometry_msgs.msg import Vector3
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from r0b0_interfaces.msg import DeviceMotion, MotorCommand, MotorCommands
else:

    class Node:
        def __init__(self, name):
            self._name = name

        def get_logger(self):
            import logging

            logging.basicConfig(level=logging.INFO)
            return logging.getLogger(self._name)

        def create_publisher(self, *args, **kwargs):
            return self

        def publish(self, *args, **kwargs):
            pass

        def destroy_node(self):
            pass

    class MultiThreadedExecutor:
        def add_node(self, node):
            pass

        def spin_once(self, *args, **kwargs):
            pass

    class _rclpy:
        def init(self, *args, **kwargs):
            pass

        def spin(self, node, *args, **kwargs):
            # keep main thread alive
            node.get_logger().info("Spinning")
            while True:
                time.sleep(1)

        def shutdown(self):
            pass

        def ok(self):
            return True

    rclpy = _rclpy()
    Vector3 = dict
    String = dict
    DeviceMotion = dict
    MotorCommands = dict
    MotorCommand = dict


from enum import Enum

import socketio
import uvicorn
from flask import Flask, jsonify, render_template, request
from flask_cors import CORS
from flask_socketio import SocketIO, emit

from r0b0.config import (
    CSR_PEM,
    KEY_PEM,
    ROOT_DIR,
    SERVER_PORT,
)
from r0b0.utils.cert_manager import ensure_https_certificates


# Set to True to disable ROS2 imports for testing without ROS2
try:
    import rclpy

    NO_ROS = False
except:
    NO_ROS = True

if not NO_ROS:
    import rclpy
    from geometry_msgs.msg import Vector3
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Int64, String

    from r0b0_interfaces.msg import DeviceMotion, MotorCommand, MotorCommands

else:

    class Node:
        def __init__(self, name):
            self._name = name

        def get_logger(self):
            import logging

            logging.basicConfig(level=logging.INFO)
            return logging.getLogger(self._name)

        def create_publisher(self, *args, **kwargs):
            return self

        def create_subscription(self, *args, **kwargs):
            return self

        def publish(self, *args, **kwargs):
            pass

        def destroy_node(self):
            pass

    class MultiThreadedExecutor:
        def add_node(self, node):
            pass

        def spin_once(self, *args, **kwargs):
            pass

    class _rclpy:
        def init(self, *args, **kwargs):
            pass

        def spin(self, node, *args, **kwargs):
            # keep main thread alive
            node.get_logger().info("Spinning")
            while True:
                time.sleep(1)

        def shutdown(self):
            pass

        def ok(self):
            return True

    rclpy = _rclpy()
    Vector3 = dict
    String = dict
    Int64 = dict
    DeviceMotion = dict
    MotorCommands = dict
    MotorCommand = dict


PAGES_FOLDER = str(ROOT_DIR / "pages" / "blsm")


class PageState(str, Enum):
    key_control = "key_control"
    calibration = "calibration"
    speech = "speech"
    sensor = "sensor"
    playback = "playback"
    keyframe_playback = "keyframe_playback"
    idle = "idle"  # default/non-configured state


_ALLOWED_STATES = {s.value for s in PageState}


class WebPageNode(Node):
    def __init__(
        self,
        name,
        template_folder: str,
        static_folder: str,
        certfile: str | None = None,
        keyfile: str | None = None,
    ):
        super().__init__(name)
        self.get_logger().info("Initializing WebPageNode...")

        # Initialize Flask app
        self.app = Flask(
            __name__,
            template_folder=template_folder,
            static_folder=static_folder,
        )
        self.app.config["TEMPLATES_AUTO_RELOAD"] = True
        self.app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0
        self.app.jinja_env.auto_reload = True

        # Allow all origins
        CORS(self.app, resources={r"/*": {"origins": "*"}})

        self.setup_routes()

        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins="*",
            max_http_buffer_size=int(1e8),
            # logger=True,
            # engineio_logger=True,
            async_mode="eventlet",
        )
        self.certfile = certfile
        self.keyfile = keyfile

    def start(self):
        """Start web server in background using eventlet greenlet."""
        eventlet.spawn(self.start_web_server)

    def start_web_server(self):
        """Start the Flask web server."""
        self.get_logger().info("Starting Flask web server...")
        self.socketio.run(
            self.app,
            host="0.0.0.0",
            port=SERVER_PORT,
            certfile=self.certfile,
            keyfile=self.keyfile,
            debug=False,
            use_reloader=False,
        )

    async def start_web_server_async(self):
        # Use ASGI app for Flask-SocketIO
        print("creating agsi_app")
        asgi_app = socketio.ASGIApp(self.socketio, self.app)
        print("created agsi_app")
        print("creating config")
        config = uvicorn.Config(
            asgi_app,
            host="0.0.0.0",
            port=SERVER_PORT,
            log_level="info",
            ssl_certfile=getattr(self, "certfile", None),
            ssl_keyfile=getattr(self, "keyfile", None),
        )

        print("creating config")
        server = uvicorn.Server(config)
        await server.serve()

    def setup_routes(self):
        """Define routes for the Flask app."""
        ...

    def _setup_route(self, _route):
        # NOTE: did the same thing back with r0b0.rigs.host
        def route_func():
            return render_template(f"{_route}.html")

        route_func.__name__ = _route
        self.app.add_url_rule(f"/{_route}", view_func=route_func)


class BlsmPageNode(WebPageNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # --- State management ---
        self.current_state = PageState.idle
        # Cache allowed state strings for O(1) membership checks
        self._allowed_states = set(_ALLOWED_STATES)
        webrtc_events = [
            "broadcaster",
            "watcher",
            "offer",
            "answer",
            "candidate",
        ]
        interface_events = [
            "phone_text",
            "device_motion",
            "key_event",
            "slider_event",
            "speech_command",
            "set_state",
            "playback_play",
            "playback_pause",
            "playback_stop",
            "pose_event",
            "keyframe_save",
            "keyframe_delete",
            "animation_save",
            "animation_play",
            "animation_pause",
            "animation_stop",
        ]
        for _event in webrtc_events + interface_events:
            self.socketio.on_event(_event, getattr(self, _event), namespace="/")
        self.broadcaster_id = None
        self.speech_commands = []
        self.pub = self.create_publisher(String, "/blsm", 10)
        self.device_motion_pub = self.create_publisher(
            DeviceMotion, "/blsm/device_motion", 10
        )
        self.key_event_pub = self.create_publisher(
            String, "/blsm/key_event", 10
        )
        self.motor_command_pub = self.create_publisher(
            MotorCommands, "/blsm/motor_cmd", 10
        )
        self.speech_command_pub = self.create_publisher(
            String, "/blsm/speech_command", 10
        )
        self.distance_sub = self.create_subscription(
            Int64,
            "/blsm/distance_sensor",
            callback=self.handle_distance,
            qos_profile=10,
        )
        self.state_pub = self.create_publisher(
            String, "/blsm/state/request", qos_profile=10
        )
        self.playback_pub = self.create_publisher(
            String, "/blsm/playback", qos_profile=10
        )
        self.keyframe_tape_pub = self.create_publisher(
            String, "/blsm/keyframe_tape", qos_profile=10
        )
        self.keyframe_playback_pub = self.create_publisher(
            String, "/blsm/keyframe_playback", qos_profile=10
        )
        self.playback_status_sub = self.create_subscription(
            String,
            "/blsm/playback/status",
            callback=self.handle_playback_status,
            qos_profile=10,
        )
        self.motor_pose_sub = self.create_subscription(
            String,
            "/blsm/motor_pose",
            callback=self.handle_motor_pose,
            qos_profile=10,
        )
        self._latest_distance = None
        self._registered_tapes: list[dict] = []

        # Keyframe animation state
        self._keyframes: list[dict] = []
        self._latest_motor_pose: dict = {
            "h": 0.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
        }
        self._current_motors: dict = {
            "1": 90.0,
            "2": 90.0,
            "3": 90.0,
            "4": 90.0,
        }
        self._keyframe_tapes: dict = {}  # name -> Tape
        self._kf_playing: bool = False
        self._kf_paused: bool = False
        self._kf_playback_greenlet = None
        self._kf_playback_gen: int = 0

        # Register example tapes for the UI
        self._load_example_tapes()

        # Setup REST API routes for state management
        self.setup_state_routes()

    def _load_example_tapes(self):
        """Load example tapes for the playback UI."""
        try:
            from r0b0.ros2.example_tapes import get_example_tapes

            for tape in get_example_tapes():
                self.register_tape(tape.name, tape.duration)
                self.get_logger().info(f"Registered tape for UI: {tape.name}")
        except Exception as e:
            self.get_logger().warning(f"Could not load example tapes: {e}")

    # -------------------- State REST API --------------------
    def setup_state_routes(self):
        @self.app.get("/api/state")
        def get_state():
            # Canonical format: { state: <string> }
            return jsonify({"state": self.current_state.value})

        @self.app.get("/api/test_sensor")
        def test_sensor():
            """Test endpoint to manually emit a sensor value."""
            test_value = random.choice(range(10))
            self.socketio.emit("sensor_value", {"value": test_value})
            return jsonify({"emitted": test_value})

        @self.app.get("/api/tapes")
        def get_tapes():
            """Get list of registered tapes for playback."""
            return jsonify({"tapes": self._registered_tapes})

        @self.app.get("/api/keyframes")
        def get_keyframes():
            """Get list of saved keyframes."""
            return jsonify({"keyframes": self._keyframes})

        @self.app.get("/api/tape/<tape_name>/frames")
        def get_tape_frames(tape_name):
            """Get all frames for a tape (for visualizer scrubbing)."""
            # Check keyframe tapes first
            if tape_name in self._keyframe_tapes:
                tape = self._keyframe_tapes[tape_name]
                frames = []
                for frame in tape.frames:
                    euler = frame.pose.rot.as_euler("ZXY")
                    frames.append(
                        {
                            "ts": frame.ts,
                            "h": frame.pose.h,
                            "yaw": float(euler[0]),
                            "pitch": float(euler[1]),
                            "roll": float(euler[2]),
                        }
                    )
                return jsonify(
                    {
                        "name": tape.name,
                        "duration": tape.duration,
                        "frames": frames,
                    }
                )

            try:
                from r0b0.ros2.example_tapes import get_example_tapes

                for tape in get_example_tapes():
                    if tape.name == tape_name:
                        frames = []
                        for frame in tape.frames:
                            euler = frame.pose.rot.as_euler("ZXY")
                            frames.append(
                                {
                                    "ts": frame.ts,
                                    "h": frame.pose.h,
                                    "yaw": float(euler[0]),
                                    "pitch": float(euler[1]),
                                    "roll": float(euler[2]),
                                }
                            )
                        return jsonify(
                            {
                                "name": tape.name,
                                "duration": tape.duration,
                                "frames": frames,
                            }
                        )
                return jsonify({"error": f"Tape '{tape_name}' not found"}), 404
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @self.app.post("/api/state")
        def set_state():
            try:
                data = request.get_json(silent=True) or {}
                print(f"{data=}")
                new_state = data.get("state")
                if not new_state:
                    return jsonify({"error": "missing state"}), 400
                # normalize
                new_state = str(new_state).strip().lower()
                # validate against enum
                if new_state not in self._allowed_states:
                    return jsonify(
                        {
                            "error": "invalid state",
                            "allowed": sorted(self._allowed_states),
                        }
                    ), 400

                # update only if changed
                if self.current_state.value != new_state:
                    self.current_state = PageState(new_state)
                    # Emit over sockets for immediate client updates
                    try:
                        self.socketio.emit(
                            "state_update",
                            {
                                "state": self.current_state.value,
                            },
                        )
                    except Exception:
                        pass

                self.state_pub.publish(String(data=self.current_state.value))
                return jsonify({"state": self.current_state.value})
            except Exception as e:
                return jsonify({"error": str(e)}), 500

    # -------------------- State Socket.IO API --------------------
    def set_state(self, data):
        try:
            data = data or {}
            print(f"socket {data=}")
            new_state = data.get("state")
            if not new_state:
                return {"error": "missing state"}
            new_state = str(new_state).strip().lower()
            if new_state not in self._allowed_states:
                return {
                    "error": "invalid state",
                    "allowed": sorted(self._allowed_states),
                }

            if self.current_state.value != new_state:
                self.current_state = PageState(new_state)
                try:
                    self.socketio.emit(
                        "state_update",
                        {
                            "state": self.current_state.value,
                        },
                    )
                except Exception:
                    pass
            self.state_pub.publish(String(data=self.current_state.value))
            return {"state": self.current_state.value}
        except Exception as e:
            return {"error": str(e)}

    def broadcaster(self, sid):
        self.get_logger().info(f"{sid=}")
        self.broadcaster_id = sid
        self.socketio.emit("broadcaster")

    def watcher(self, sid):
        for i in range(5):
            self.get_logger().info(f"Watcher attempt {i}")
            try:
                self.socketio.emit(
                    "watcher",
                    sid,
                    to=self.broadcaster_id,
                )
                self.get_logger().info(f"{self.broadcaster_id=}")
                self.get_logger().info("watcher")
                self.pub.publish(String(data="watcher"))
                return
            except:
                pass

    def offer(self, sid, msg, *args, **kwargs):
        self.socketio.emit(
            "offer",
            (sid, msg),
            to=sid,
        )
        self.get_logger().info("offer")

    def answer(self, sid, msg):
        # TODO: handle max connections
        self.socketio.emit(
            "answer",
            (sid, msg),
            to=sid,
        )

    def candidate(self, sid, msg):
        self.socketio.emit(
            "candidate",
            (sid, msg),
            to=sid,
        )

    def phone_text(self, *args, **kwargs):
        print(args, kwargs)
        ...
        # breakpoint()

    def key_event(self, msg, **kwargs):
        print(msg)
        self.key_event_pub.publish(String(data=msg["code"]))

    def device_motion(self, msg):
        self.device_motion_pub.publish(
            DeviceMotion(
                xyz=Vector3(**{k: float(msg[k]) for k in ["x", "y", "z"]}),
                yaw_offset=float(msg["yaw"]),
                ears=msg["ears"],
                portrait=msg["portrait"],
                mirror=msg["mirror"],
            )
        )

    def slider_event(self, msg):
        if "id" in msg and "value" in msg:
            print(msg)
            self._current_motors[str(msg["id"])] = float(msg["value"])
            self.motor_command_pub.publish(
                MotorCommands(
                    data=[
                        MotorCommand(
                            name=msg["id"], position_rad=float(msg["value"])
                        )
                    ]
                )
            )

    def speech_command(self, data):
        """Handle speech command from client."""
        try:
            command = data.get("command", "")
            language = data.get("language", "en-US")
            confidence = data.get("confidence", 0.0)

            if not command:
                emit(
                    "speech_response",
                    {"status": "error", "message": "No command provided"},
                )
                return

            # Store the command
            self.speech_commands.append(data)

            # Publish to ROS2 topic
            ros_msg = String(data=command)
            self.speech_command_pub.publish(ros_msg)

            self.get_logger().info(
                f"Speech command published: {command} (lang: {language}, conf: {confidence:.2f})"
            )

            # Send success response
            emit(
                "speech_response",
                {
                    "status": "success",
                    "message": f"Command processed: {command}",
                    "command": command,
                    "language": language,
                    "confidence": confidence,
                },
            )

        except Exception as e:
            self.get_logger().error(f"Error processing speech command: {e}")
            emit(
                "speech_response",
                {
                    "status": "error",
                    "message": f"Error processing command: {str(e)}",
                },
            )

    def handle_distance(self, msg: Int64):
        """ROS2 callback - emit sensor value directly to clients."""
        self._latest_distance = msg.data
        self.socketio.emit("sensor_value", {"value": msg.data})
        print(msg.data)

    def handle_playback_status(self, msg: String):
        """ROS2 callback - emit playback status to clients.

        When the local keyframe greenlet is running, suppress status from the
        robot node (its setup() fires a spurious "stopped" right after the state
        change, which would immediately reset the browser UI to "stopped").
        """

        # If our local greenlet owns the playback, ignore robot-side status
        # to avoid the setup()-induced "stopped" overwriting the "playing" state.
        if self._kf_playing:
            return

        try:
            status = json.loads(msg.data)

            # Emit main status update
            self.socketio.emit(
                "playback_status",
                {
                    "state": status.get("state", "stopped"),
                    "tape_name": status.get("tape_name", ""),
                    "progress": status.get("progress", 0.0),
                    "current_time": status.get("current_time", 0.0),
                    "total_time": status.get("total_time", 0.0),
                },
            )

            # Emit pose update if available
            pose = status.get("pose")
            if pose:
                self.socketio.emit(
                    "playback_pose",
                    {
                        "h": pose.get("h", 0),
                        "yaw": pose.get("yaw", 0),
                        "pitch": pose.get("pitch", 0),
                        "roll": pose.get("roll", 0),
                        "progress": status.get("progress", 0.0),
                    },
                )
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid playback status JSON: {msg.data}")

    def handle_motor_pose(self, msg: String):
        """ROS2 callback - emit motor pose to clients for visualizer.

        Idle/Breathe state → ``idle_pose`` event (only index.html listens).
        All other states  → ``motor_pose`` event (every page can listen).
        """
        import json

        try:
            pose = json.loads(msg.data)
            state = pose.pop("state", "")
            if state == "idle":
                self.socketio.emit("idle_pose", pose)
            else:
                self.socketio.emit("motor_pose", pose)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid motor pose JSON: {msg.data}")

    # -------------------- Playback Socket.IO API --------------------
    def playback_play(self, data):
        """Handle playback play command from client."""
        tape_name = data.get("tape_name", "")
        loop = data.get("loop", False)
        if not tape_name:
            return {"error": "missing tape_name"}

        # Publish to ROS2 topic as JSON command
        import json

        cmd = json.dumps(
            {"action": "play", "tape_name": tape_name, "loop": loop}
        )
        self.playback_pub.publish(String(data=cmd))
        self.get_logger().info(f"Playback play: {tape_name} (loop={loop})")

        # Emit status update to all clients
        self.socketio.emit(
            "playback_status",
            {
                "state": "playing",
                "tape_name": tape_name,
            },
        )
        return {"status": "ok"}

    def playback_pause(self, data):
        """Handle playback pause command from client."""
        import json

        cmd = json.dumps({"action": "pause"})
        self.playback_pub.publish(String(data=cmd))
        self.get_logger().info("Playback pause")

        self.socketio.emit("playback_status", {"state": "paused"})
        return {"status": "ok"}

    def playback_stop(self, data):
        """Handle playback stop command from client."""
        import json

        cmd = json.dumps({"action": "stop"})
        self.playback_pub.publish(String(data=cmd))
        self.get_logger().info("Playback stop")

        self.socketio.emit("playback_status", {"state": "stopped"})
        return {"status": "ok"}

    # -------------------- Keyframe Socket.IO API --------------------
    def pose_event(self, data):
        """Receive a pose from the keyframe page sliders, track it, and relay
        it as motor_pose so the floating visualizer and other tabs stay in sync."""
        pose = {k: float(data.get(k, 0)) for k in ("h", "yaw", "pitch", "roll")}
        self._latest_motor_pose = pose
        self.socketio.emit("motor_pose", pose)

    def keyframe_save(self, data):
        """Save current motor pose as a named keyframe."""
        from datetime import datetime

        name = (
            data.get("name") or f"kf_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        # Avoid duplicate names
        existing_names = {kf["name"] for kf in self._keyframes}
        if name in existing_names:
            suffix = 1
            base = name
            while name in existing_names:
                name = f"{base}_{suffix}"
                suffix += 1
        kf = {
            "name": name,
            "motors": dict(self._current_motors),
            "pose": dict(self._latest_motor_pose),
            "created_at": time.time(),
        }
        self._keyframes.append(kf)
        emit("keyframe_saved", kf)
        self.get_logger().info(f"Saved keyframe: {name}")
        return kf

    def keyframe_delete(self, data):
        """Delete a keyframe by name."""
        name = data.get("name")
        self._keyframes = [kf for kf in self._keyframes if kf["name"] != name]
        emit("keyframe_deleted", {"name": name})
        return {"status": "ok"}

    def animation_save(self, data):
        """Build a Tape from a keyframe timeline and register it."""
        import json
        from datetime import datetime
        from r0b0.ros2.tapes import Tape, Frame
        from r0b0.ros2.pose import BlsmPose
        from scipy.spatial.transform import Rotation as _Rotation

        anim_name = (
            data.get("name")
            or f"anim_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        timeline = data.get("frames", [])  # [{keyframe_name, ts}]

        kf_map = {kf["name"]: kf for kf in self._keyframes}
        tape_frames = []
        for entry in sorted(timeline, key=lambda x: float(x.get("ts", 0))):
            kf = kf_map.get(entry.get("keyframe_name"))
            if not kf:
                continue
            p = kf["pose"]
            rot = _Rotation.from_euler("ZXY", [p["yaw"], p["pitch"], p["roll"]])
            tape_frames.append(
                Frame(ts=float(entry["ts"]), pose=BlsmPose(h=p["h"], rot=rot))
            )

        if not tape_frames:
            emit(
                "animation_save_error", {"error": "No valid frames in timeline"}
            )
            return {"error": "no frames"}

        tape = Tape(name=anim_name, frames=tape_frames)
        self._keyframe_tapes[anim_name] = tape
        self.register_tape(anim_name, tape.duration)

        # Serialize and publish to robot node so it can register the tape
        serialized_frames = []
        for f in tape.frames:
            euler = f.pose.rot.as_euler("ZXY")
            serialized_frames.append(
                {
                    "ts": float(f.ts),
                    "h": float(f.pose.h),
                    "yaw": float(euler[0]),
                    "pitch": float(euler[1]),
                    "roll": float(euler[2]),
                }
            )
        tape_json = json.dumps({"name": anim_name, "frames": serialized_frames})
        self.keyframe_tape_pub.publish(String(data=tape_json))

        emit("animation_saved", {"name": anim_name, "duration": tape.duration})
        self.get_logger().info(
            f"Saved animation '{anim_name}' with {len(tape_frames)} frames"
        )
        return {"name": anim_name, "duration": tape.duration}

    def animation_play(self, data):
        """Start keyframe animation playback on both the robot and local visualizer."""
        tape_name = data.get("tape_name", "")
        loop = data.get("loop", False)
        tape = self._keyframe_tapes.get(tape_name)
        if not tape:
            self.get_logger().warn(
                f"animation_play: tape '{tape_name}' not found in {list(self._keyframe_tapes.keys())}"
            )
            emit(
                "playback_status", {"state": "stopped", "tape_name": tape_name}
            )
            emit(
                "kf_playback_error",
                {
                    "error": f"Animation '{tape_name}' not found. Try saving it again."
                },
            )
            return {"error": "tape not found"}

        # Switch robot to KEYFRAME_PLAYBACK state and send play command
        self.state_pub.publish(String(data="keyframe_playback"))
        cmd = json.dumps(
            {"action": "play", "tape_name": tape_name, "loop": loop}
        )
        self.keyframe_playback_pub.publish(String(data=cmd))
        self.get_logger().info(
            f"Keyframe animation play: '{tape_name}' (loop={loop})"
        )

        # Stop any currently running greenlet, then start fresh.
        # _kf_playing must be True when the new greenlet starts — set it only
        # AFTER the cleanup block, never inside it.
        if self._kf_playing:
            self._kf_playing = False
            eventlet.sleep(
                0.1
            )  # allow old greenlet's current iteration to finish

        tape.loop = loop
        tape.reset()
        self._kf_paused = False
        self._kf_playback_gen += 1
        current_gen = self._kf_playback_gen
        self._kf_playing = True
        print(
            f"[kf] animation_play: spawning gen={current_gen}, _kf_playing={self._kf_playing}, tape={tape_name}, frames={len(tape.frames)}, duration={tape.duration:.2f}s"
        )
        self._kf_playback_greenlet = eventlet.spawn(
            self._run_kf_playback, tape_name, current_gen
        )
        return {"status": "ok"}

    def animation_pause(self, data):
        """Pause/resume keyframe animation playback on robot and local visualizer."""
        self._kf_paused = not self._kf_paused
        state = "paused" if self._kf_paused else "playing"
        cmd = json.dumps({"action": "pause" if self._kf_paused else "play"})
        self.keyframe_playback_pub.publish(String(data=cmd))
        self.socketio.emit("playback_status", {"state": state})
        return {"status": "ok"}

    def animation_stop(self, data):
        """Stop keyframe animation playback on robot and local visualizer."""
        self._kf_playing = False
        self._kf_paused = False
        cmd = json.dumps({"action": "stop"})
        self.keyframe_playback_pub.publish(String(data=cmd))
        self.state_pub.publish(String(data="idle"))
        self.socketio.emit("playback_status", {"state": "stopped"})
        return {"status": "ok"}

    def _run_kf_playback(self, tape_name: str, gen: int):
        """Background greenlet: emit pose events for keyframe animation preview."""
        print(
            f"[kf] _run_kf_playback: entered gen={gen}, _kf_playing={self._kf_playing}, current_gen={self._kf_playback_gen}"
        )
        tape = self._keyframe_tapes.get(tape_name)
        if not tape:
            print(f"[kf] _run_kf_playback: tape '{tape_name}' not found")
            self.get_logger().error(
                f"_run_kf_playback: tape '{tape_name}' not found"
            )
            self.socketio.emit(
                "playback_status", {"state": "stopped", "tape_name": tape_name}
            )
            return

        print(
            f"[kf] _run_kf_playback: tape found, duration={tape.duration:.2f}s, frames={len(tape.frames)}"
        )
        self.get_logger().info(
            f"_run_kf_playback: starting '{tape_name}' gen={gen} (duration={tape.duration:.2f}s, frames={len(tape.frames)})"
        )
        self.socketio.emit(
            "playback_status",
            {
                "state": "playing",
                "tape_name": tape_name,
                "total_time": tape.duration,
            },
        )

        iters = 0
        exit_reason = "unknown"
        try:
            while self._kf_playing and self._kf_playback_gen == gen:
                iters += 1
                if self._kf_paused:
                    eventlet.sleep(0.05)
                    continue

                pose = tape.get_frame_at_ts(time.time())
                if pose is None:
                    exit_reason = "pose_is_none"
                    break

                euler = pose.rot.as_euler("ZXY")
                progress = tape.get_progress()
                pose_data = {
                    "h": float(pose.h),
                    "yaw": float(euler[0]),
                    "pitch": float(euler[1]),
                    "roll": float(euler[2]),
                }
                self.socketio.emit("motor_pose", pose_data)
                self.socketio.emit(
                    "playback_pose", {**pose_data, "progress": progress}
                )
                self.socketio.emit(
                    "playback_status",
                    {
                        "state": "playing",
                        "tape_name": tape_name,
                        "progress": progress,
                        "current_time": progress * tape.duration,
                        "total_time": tape.duration,
                    },
                )
                eventlet.sleep(0.05)

                # Natural completion for non-looping tapes.
                # get_frame_at_ts never returns None once past the end — it holds on the
                # last frame — so we check progress explicitly to stop the loop.
                if not tape.loop and progress >= 1.0:
                    exit_reason = "completed"
                    break
            else:
                exit_reason = f"while_false: _kf_playing={self._kf_playing}, gen_match={self._kf_playback_gen == gen}(cur={self._kf_playback_gen},mine={gen})"
        except Exception as exc:
            exit_reason = f"exception: {exc}"
            self.get_logger().error(f"_run_kf_playback error: {exc}")
            self.socketio.emit("kf_playback_error", {"error": str(exc)})
        finally:
            print(
                f"[kf] _run_kf_playback: exiting after {iters} iters, reason={exit_reason}, gen={gen}, current_gen={self._kf_playback_gen}, _kf_playing={self._kf_playing}"
            )
            # Only reset shared state if we are still the current generation.
            # A newer greenlet may have already taken over — don't clobber its state.
            if self._kf_playback_gen == gen:
                self._kf_playing = False
                self.socketio.emit(
                    "playback_status",
                    {
                        "state": "stopped",
                        "tape_name": tape_name,
                        "progress": 0.0,
                    },
                )

    def register_tape(self, name: str, duration: float = 0.0):
        """Register a tape for the playback UI."""
        # Avoid duplicates
        for tape in self._registered_tapes:
            if tape["name"] == name:
                tape["duration"] = duration
                return
        self._registered_tapes.append({"name": name, "duration": duration})

    def setup_routes(self):
        """Define routes for the Flask app."""

        @self.app.route("/")
        def home():
            return render_template("index.html")

        @self.app.route("/speech_recognition")
        def speech_recognition():
            return render_template("speech_recognition.html")

        @self.app.route("/speech_history_page")
        def speech_history_page():
            return render_template("speech_history.html")

        @self.app.route("/speech_history")
        def speech_history():
            from flask import jsonify

            return jsonify(self.speech_commands)

        for _route in [
            "blsm_controller",
            "blsm_player",
            "blsm_playback",
            "blsm_broadcast",
            "blsm_web",
            "blsm_sensor",
            "blsm_calib",
            "blsm_keyframes",
            "reset",
            "tapes",
        ]:
            # NOTE: must call in separate function or else
            # the for-loop does not recreate new method instances
            self._setup_route(_route)


def main(args=None):
    rclpy.init(args=args)

    # Ensure HTTPS certificates exist (auto-generate if needed)
    try:
        cert_path, key_path = ensure_https_certificates(CSR_PEM, KEY_PEM)
        print(f"✓ HTTPS enabled with certificates: {cert_path}")
    except Exception as e:
        print(f"⚠ Warning: Could not set up HTTPS certificates: {e}")
        print("  Continuing without HTTPS...")
        cert_path, key_path = None, None

    page_kwargs = {
        "name": "web_page_node",
        "template_folder": os.path.join(PAGES_FOLDER, "templates"),
        "static_folder": os.path.join(PAGES_FOLDER, "static"),
    }
    if cert_path and key_path:
        page_kwargs.update(
            {"certfile": str(cert_path), "keyfile": str(key_path)}
        )

    node = BlsmPageNode(**page_kwargs)

    # node = SliderPageNode("slider_page_node",
    #                       template_folder=os.path.abspath(
    #                           ROOT_DIR / "pages" / "slider" / "templates"),
    #                       static_folder=os.path.abspath(
    #                           ROOT_DIR / "pages" / "slider" / "static"),
    #                       certfile=CSR_PEM, keyfile=KEY_PEM)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # Start web server in background (non-blocking)
    node.start()

    try:
        # Use spin_once with eventlet.sleep to allow greenlets to run
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)
            eventlet.sleep(0.01)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WebPageNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
