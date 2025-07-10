import time
from rclpy.executors import MultiThreadedExecutor
import copy
import os
from threading import Thread
from r0b0.config import SOCKET_ADDR, LOCALHOST, SERVER_PORT, ROOT_DIR, CSR_PEM, KEY_PEM

import rclpy
from flask import Flask, render_template, request
from rclpy.node import Node
from std_msgs.msg import String
from flask_socketio import SocketIO, emit
from flask_cors import CORS


PAGES_FOLDER = str(ROOT_DIR / "pages" / "blsm")


class WebPageNode(Node):
    def __init__(self, name, certfile: str, keyfile: str):
        super().__init__(name)
        self.get_logger().info("Initializing WebPageNode...")

        # Initialize Flask app
        self.app = Flask(
            __name__,
            template_folder=os.path.join(PAGES_FOLDER, "templates"),
            static_folder=os.path.join(PAGES_FOLDER, "static"),
        )
        CORS(self.app)
        self.setup_routes()

        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins=[
                "*",
                SOCKET_ADDR,
                f"https://{LOCALHOST}:{SERVER_PORT}",
                f"https://{LOCALHOST}"
            ],
            max_http_buffer_size=1e8,
        )
        self.socketio.on_event("test", print, namespace="/")

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
        self.socketio.run(self.app, host="0.0.0.0", port=SERVER_PORT,
                          certfile=self.certfile, keyfile=self.keyfile)


class BlsmPageNode(WebPageNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        webrtc_events = ["broadcaster", "watcher",
                         "offer", "answer", "candidate"]
        interface_events = ["phone_text"]
        for _event in webrtc_events+interface_events:
            self.socketio.on_event(
                _event, getattr(self, _event), namespace="/")
        self.broadcaster_id = None
        self.pub = self.create_publisher(String, "/blsm", 10)
        self.server_thread = Thread(target=self.start_web_server)

    def broadcaster(self, sid):
        self.broadcaster_id = sid
        self.socketio.emit(self, "broadcaster")
        self.get_logger().info("broadcaster")

    def watcher(self, sid):
        if not self.broadcaster_id:
            return
        self.socketio.emit(
            self,
            "watcher",
            request.sid,
            to=self.broadcaster_id,
        )
        self.get_logger().info("watcher")
        self.pub.publish(String(data="watcher"))

    def offer(self, sid, msg, *args, **kwargs):
        self.socketio.emit(
            self,
            "offer",
            (request.sid, msg),
            to=sid,
        )
        self.get_logger().info("offer")

    def answer(self, sid, msg):
        # TODO: handle max connections
        self.socketio.emit(
            self,
            "answer",
            (request.sid, msg),
            to=sid,
        )

    def candidate(self, sid, msg):
        self.socketio.emit(
            self,
            "candidate",
            (request.sid, msg),
            to=sid,
        )

    def phone_text(self, *args, **kwargs):
        breakpoint()

    def setup_routes(self):
        """Define routes for the Flask app."""

        for _route in ["blsm_controller", "blsm_player", "blsm_broadcast"]:
            # NOTE: must call in separate function or else
            # the for-loop does not recreate new method instances
            self._setup_route(_route)

    def _setup_route(self, _route):
        # NOTE: did the same thing back with r0b0.rigs.host
        def route_func(): return render_template(f"{_route}.html")
        route_func.__name__ = _route
        self.app.add_url_rule(
            f"/{_route}", view_func=route_func)


def main(args=None):
    rclpy.init(args=args)
    # node = WebPageNode("web_page_node")
    node = BlsmPageNode("web_page_node", certfile=CSR_PEM, keyfile=KEY_PEM)
    # Use a MultiThreadedExecutor for non-blocking spinning
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.start()

    try:
        while rclpy.ok():
            # Spin once to process callbacks
            executor.spin_once(timeout_sec=0.1)

            # Perform other tasks here if needed
            time.sleep(0.1)
            # node.pub.publish(String(data="blsm"))
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WebPageNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
