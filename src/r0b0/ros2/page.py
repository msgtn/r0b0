"""
On computer, go to https://localhost:8080/blsm_broadcast
On mobile, go to https://r0b0.ngrok.io/blsm_controller
"""

import copy
import os
import time
from threading import Thread

import rclpy
from flask import Flask, render_template, request
from flask_cors import CORS
from flask_socketio import SocketIO, emit
from geometry_msgs.msg import Vector3
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from r0b0.config import (
    CSR_PEM,
    KEY_PEM,
    LOCALHOST,
    ROOT_DIR,
    SERVER_PORT,
    SOCKET_ADDR,
)
from r0b0_interfaces.msg import DeviceMotion

BLSM_PAGES_FOLDER = str(ROOT_DIR / "pages" / "blsm")


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
            max_http_buffer_size=1e8,
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
        interface_events = ["phone_text", "device_motion", "key_event"]
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
        self.server_thread = Thread(target=self.start_web_server)

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

    def setup_routes(self):
        """Define routes for the Flask app."""

        for _route in [
            "blsm_controller",
            "blsm_player",
            "blsm_broadcast",
            "blsm_web",
            "reset",
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
