"""
On computer, go to https://localhost:8080/blsm_broadcast
On mobile, go to https://r0b0.ngrok.io/blsm_controller
"""

import os
import time
from threading import Thread

# Set to True to disable ROS2 imports for testing without ROS2
NO_ROS = True

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
            node.get_logger().info('Spinning')
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


from flask import Flask, render_template, request
from flask_cors import CORS
from flask_socketio import SocketIO, emit

from r0b0.config import (
    CSR_PEM,
    KEY_PEM,
    LOCALHOST,
    ROOT_DIR,
    SERVER_PORT,
    SOCKET_ADDR,
)
if not NO_ROS:
    from r0b0_interfaces.msg import DeviceMotion, MotorCommands, MotorCommand

BLSM_PAGES_FOLDER = str(ROOT_DIR / "pages" / "blsm")
MAIN_PAGES_FOLDER = str(ROOT_DIR / "pages" / "main")


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
        )
        self.certfile = certfile
        self.keyfile = keyfile
        # Start Flask server in a separate thread
        self.server_thread = Thread(target=self.start_web_server)

    def start(self):
        self.server_thread.start()

    def setup_routes(self):
        """Define routes for the Flask app."""
        ...

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

    def _setup_route(self, _route):
        # NOTE: did the same thing back with r0b0.rigs.host
        def route_func():
            return render_template(f"{_route}.html")

        route_func.__name__ = _route
        self.app.add_url_rule(f"/{_route}", view_func=route_func)


class BlsmPageNode(WebPageNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
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
        ]
        for _event in webrtc_events + interface_events:
            self.socketio.on_event(_event, getattr(self, _event), namespace="/")
        self.broadcaster_id = None
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
        self.server_thread = Thread(target=self.start_web_server)
        
        # Add support for serving main static files
        self.setup_main_static_route()

    def setup_main_static_route(self):
        """Set up route to serve static files from main pages folder."""
        from flask import send_from_directory
        
        @self.app.route('/main/<path:filename>')
        def main_static(filename):
            return send_from_directory(
                os.path.join(MAIN_PAGES_FOLDER, 'static'), 
                filename
            )

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
                    request.sid,
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
            (request.sid, msg),
            to=sid,
        )
        self.get_logger().info("offer")

    def answer(self, sid, msg):
        # TODO: handle max connections
        self.socketio.emit(
            "answer",
            (request.sid, msg),
            to=sid,
        )

    def candidate(self, sid, msg):
        self.socketio.emit(
            "candidate",
            (request.sid, msg),
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

    def setup_routes(self):
        """Define routes for the Flask app."""
        
        # Add home page route to serve main template
        @self.app.route('/')
        def home():
            from flask import render_template_string
            # Read the main template file directly
            main_template_path = os.path.join(MAIN_PAGES_FOLDER, 'templates', 'index.html')
            with open(main_template_path, 'r') as f:
                template_content = f.read()
            # Replace asset paths to use the /main/ prefix
            template_content = template_content.replace('href="css/', 'href="/main/css/')
            template_content = template_content.replace('src="js/', 'src="/main/js/')
            template_content = template_content.replace('src="assets/', 'src="/main/assets/')
            template_content = template_content.replace('href="assets/', 'href="/main/assets/')
            return render_template_string(template_content)

        for _route in [
            "blsm_controller",
            "blsm_player",
            "blsm_broadcast",
            "blsm_web",
            "blsm_calib",
            "reset",
            "tapes",
        ]:
            # NOTE: must call in separate function or else
            # the for-loop does not recreate new method instances
            self._setup_route(_route)


class SliderPageNode(WebPageNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        events = ["slider"]
        for _event in events:
            self.socketio.on_event(_event, getattr(self, _event), namespace="/")
            self.pub = self.create_publisher(String, "/blsm", 10)
            self.server_thread = Thread(target=self.start_web_server)

    def slider(self, *args, **kwargs):
        breakpoint()

    def setup_routes(self):
        """Define routes for the Flask app."""

        for _route in ["slider"]:
            # NOTE: must call in separate function or else
            # the for-loop does not recreate new method instances
            self._setup_route(_route)


def main(args=None):
    rclpy.init(args=args)
    page_kwargs = {
        "name": "web_page_node",
        "template_folder": os.path.join(BLSM_PAGES_FOLDER, "templates"),
        "static_folder": os.path.join(BLSM_PAGES_FOLDER, "static"),
    }
    if os.path.exists(CSR_PEM) and os.path.exists(KEY_PEM):
        page_kwargs.update({"certfile": CSR_PEM, "keyfile": KEY_PEM})

    node = BlsmPageNode(**page_kwargs)

    # node = SliderPageNode("slider_page_node",
    #                       template_folder=os.path.abspath(
    #                           ROOT_DIR / "pages" / "slider" / "templates"),
    #                       static_folder=os.path.abspath(
    #                           ROOT_DIR / "pages" / "slider" / "static"),
    #                       certfile=CSR_PEM, keyfile=KEY_PEM)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.start()

    try:
        rclpy.spin(node)
        # while rclpy.ok():
        #     #     # Spin once to process callbacks
        #     executor.spin_once(timeout_sec=0)

        #     # Perform other tasks here if needed
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WebPageNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
