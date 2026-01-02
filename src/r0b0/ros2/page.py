"""
On computer, go to https://localhost:8080/blsm_broadcast
On mobile, go to https://r0b0.ngrok.io/blsm_controller
"""

import eventlet
import random

eventlet.monkey_patch()

import os
import time
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


BLSM_PAGES_FOLDER = str(ROOT_DIR / "pages" / "blsm")
MAIN_PAGES_FOLDER = str(ROOT_DIR / "pages" / "main")


class PageState(str, Enum):
    key_control = "key_control"
    calibration = "calibration"
    speech = "speech"
    sensor = "sensor"
    playback = "playback"
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
        self.playback_status_sub = self.create_subscription(
            String,
            "/blsm/playback/status",
            callback=self.handle_playback_status,
            qos_profile=10,
        )
        self._latest_distance = None
        self._registered_tapes: list[dict] = []

        # Register example tapes for the UI
        self._load_example_tapes()

        # Add support for serving main static files
        self.setup_main_static_route()
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

    def setup_main_static_route(self):
        """Set up route to serve static files from main pages folder."""
        from flask import send_from_directory

        @self.app.route("/main/<path:filename>")
        def main_static(filename):
            return send_from_directory(
                os.path.join(MAIN_PAGES_FOLDER, "static"), filename
            )

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

        @self.app.get("/api/tape/<tape_name>/frames")
        def get_tape_frames(tape_name):
            """Get all frames for a tape (for visualizer scrubbing)."""
            try:
                from r0b0.ros2.example_tapes import get_example_tapes

                for tape in get_example_tapes():
                    if tape.name == tape_name:
                        frames = []
                        for frame in tape.frames:
                            euler = frame.pose.rot.as_euler("ZXY")
                            frames.append({
                                "ts": frame.ts,
                                "h": frame.pose.h,
                                "yaw": float(euler[0]),
                                "pitch": float(euler[1]),
                                "roll": float(euler[2]),
                            })
                        return jsonify({
                            "name": tape.name,
                            "duration": tape.duration,
                            "frames": frames,
                        })
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
        """ROS2 callback - emit playback status to clients."""
        import json

        try:
            status = json.loads(msg.data)

            # Emit main status update
            self.socketio.emit("playback_status", {
                "state": status.get("state", "stopped"),
                "tape_name": status.get("tape_name", ""),
                "progress": status.get("progress", 0.0),
                "current_time": status.get("current_time", 0.0),
                "total_time": status.get("total_time", 0.0),
            })

            # Emit pose update if available
            pose = status.get("pose")
            if pose:
                self.socketio.emit("playback_pose", {
                    "h": pose.get("h", 0),
                    "yaw": pose.get("yaw", 0),
                    "pitch": pose.get("pitch", 0),
                    "roll": pose.get("roll", 0),
                    "progress": status.get("progress", 0.0),
                })
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid playback status JSON: {msg.data}")

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

        # Add home page route to serve main template
        @self.app.route("/")
        def home():
            from flask import render_template_string

            # Read the main template file directly
            main_template_path = os.path.join(
                MAIN_PAGES_FOLDER, "templates", "index.html"
            )
            with open(main_template_path, "r") as f:
                template_content = f.read()
            # Replace asset paths to use the /main/ prefix
            template_content = template_content.replace(
                'href="css/', 'href="/main/css/'
            )
            template_content = template_content.replace(
                'src="js/', 'src="/main/js/'
            )
            template_content = template_content.replace(
                'src="assets/', 'src="/main/assets/'
            )
            template_content = template_content.replace(
                'href="assets/', 'href="/main/assets/'
            )
            return render_template_string(template_content)

        # Add speech recognition route from main templates
        @self.app.route("/speech_recognition")
        def speech_recognition():
            from flask import render_template_string

            # Read the speech recognition template file directly
            speech_template_path = os.path.join(
                MAIN_PAGES_FOLDER, "templates", "speech_recognition.html"
            )
            with open(speech_template_path, "r") as f:
                template_content = f.read()
            # Replace asset paths to use the /main/ prefix
            template_content = template_content.replace(
                'href="css/', 'href="/main/css/'
            )
            template_content = template_content.replace(
                'src="js/', 'src="/main/js/'
            )
            template_content = template_content.replace(
                'src="assets/', 'src="/main/assets/'
            )
            template_content = template_content.replace(
                'href="assets/', 'href="/main/assets/'
            )
            return render_template_string(template_content)

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
        "template_folder": os.path.join(BLSM_PAGES_FOLDER, "templates"),
        "static_folder": os.path.join(BLSM_PAGES_FOLDER, "static"),
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
