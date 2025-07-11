"""
On computer, go to https://localhost:8080/blsm_broadcast
On mobile, go to https://r0b0.ngrok.io/blsm_controller
"""

import copy
import os
import time
from abc import abstractmethod
from threading import Thread
import numpy as np
from typing_extensions import override

import rclpy
import serial
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from r0b0.config import (
    LOCALHOST,
    ROOT_DIR,
    SERVER_PORT,
    SOCKET_ADDR,
)
from r0b0_interfaces.msg import DeviceMotion

BLSM_PAGES_FOLDER = str(ROOT_DIR / "pages" / "blsm")


class RobotNode(Node):
    def __init__(
        self,
        name,
    ):
        super().__init__(name)
        self.motors: dict[str, float] = {}

        self.create_timer(
            0.01,
            callback=self.write_motors,
        )

    @abstractmethod
    def write_motors(self): ...


class SerialRobotNode(RobotNode):
    def __init__(self, name, port="/dev/ttyACM0", baudrate=115200):
        super().__init__(name)
        self.serial = serial.Serial(port, baudrate)

    @override
    def write_motors(self):
        """Send the motor values as a string"""
        params: str = "&".join(["=".join([str(k), str(v)])
                               for k, v in self.motors.items()])
        params += "\n"
        self.serial.write(bytes(params, encoding="utf-8"))


class BlsmRobotNode(SerialRobotNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.device_motion_sub = self.create_subscription(
            DeviceMotion, "/blsm/device_motion", callback=self.ik, qos_profile=10)

    def ik(self, msg):
        self.get_logger().info(f"{msg=}")
        self.motors.update({1: int(np.interp(msg.xyz.x, [0, 6], [10, 170]))})
        self.get_logger().info(f"{self.motors=}")


def main(args=None):
    rclpy.init(args=args)
    node = BlsmRobotNode("robot_node")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            # Spin once to process callbacks
            executor.spin_once(timeout_sec=0.1)

            # Perform other tasks here if needed
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WebPageNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
