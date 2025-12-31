"""
On computer, go to https://localhost:8080/blsm_broadcast
On mobile, go to https://r0b0.ngrok.io/blsm_controller
"""

import os
import time
from abc import abstractmethod
from enum import Enum

import numpy as np
import rclpy
import serial
from geometry_msgs.msg import Vector3
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int64, String
from typing_extensions import override

from r0b0.config import (
    ROOT_DIR,
)
from r0b0.ros2.actions import (
    BlsmAction,
    Breathe,
    Keyboard,
    Phone,
    Playback,
    Sensor,
    Slider,
)
from r0b0_interfaces.msg import DeviceMotion, MotorCommand, MotorCommands

BLSM_PAGES_FOLDER = str(ROOT_DIR / "pages" / "blsm")


class RobotNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name)
        self.motor_id_pos: dict[str, float] = {}
        self.rotation = Rotation.from_euler("ZXY", angles=[0, 0, 0])
        self.states = []

        # self.create_timer(
        #     0.1,
        #     callback=self.write_motors,
        # )

    @abstractmethod
    def write_motors(self): ...

    @abstractmethod
    def read_serial(self): ...


class SerialRobotNode(RobotNode):
    def __init__(self, name, port="/dev/ttyACM0", baudrate=115200):
        super().__init__(name)
        self.serial = serial.Serial(port, baudrate)

    @override
    def write_motors(self):
        """Send the motor values as a string"""
        # params: str = "&".join(["=".join([str(k), str(v)])
        params: str = "&".join([f"{k}={v:0.2f}" for k, v in self.motor_id_pos.items()])
        params += "\n"
        self.serial.write(bytes(params, encoding="utf-8"))


class StateEnum(Enum):
    IDLE = 0
    SENSOR = 1
    PLAYBACK = 2
    CONVERSATION = 3
    KEY_CONTROL = 4
    PHONE = 5
    CALIBRATION = 6


DEG2DXL = [
    [[-10, 140], [0, 2048]],
    [[-10, 140], [0, 2048]],
    [[-10, 140], [0, 2048]],
    [[-140, 140], [0, 4096]],
]
DEG2SERVO = [
    [[-10, 140], [90, 170]],
    [[-10, 140], [90, 170]],
    [[-10, 140], [90, 170]],
    [[-140, 140], [10, 170]],
]


class BlsmRobotNode(SerialRobotNode):
    def __init__(self, motor_map=DEG2DXL, **kwargs):
        super().__init__(**kwargs)
        self._motor_map = motor_map
        self.phone_action = Phone()
        self.device_motion_sub = self.create_subscription(
            DeviceMotion,
            "/blsm/device_motion",
            callback=self.phone_action.ik_from_msg,
            qos_profile=10,
        )
        self.keyboard_action = Keyboard()
        self.key_event_sub = self.create_subscription(
            String,
            "/blsm/key_event",
            callback=self.keyboard_action.update_rotation_from_keys,
            qos_profile=10,
        )
        self.slider_action = Slider()
        self.slider_event_sub = self.create_subscription(
            MotorCommands,
            "/blsm/motor_cmd",
            callback=self.slider_action.update_motors_from_sliders,
            qos_profile=10,
        )
        self.distance_pub = self.create_publisher(
            Int64, "/blsm/distance_sensor", qos_profile=10
        )
        self.state_sub = self.create_subscription(
            String,
            "/blsm/state/request",
            callback=self.callback_state,
            qos_profile=10,
        )
        self.playback_action = Playback()
        self.playback_request_sub = self.create_subscription(
            String,
            "/blsm/playback/request",
            callback=self.playback_action.playback_callback,
            qos_profile=10,
        )

        self.h: float = 0
        self.alpha: float = 0
        self.yaw_offset = 0
        self.mirror: bool = True
        self.sensitivity: float = 1

        self.states: dict[StateEnum, BlsmAction] = {
            StateEnum.IDLE: Breathe(),
            StateEnum.SENSOR: Sensor(
                serial=self.serial, distance_pub=self.distance_pub
            ),
            StateEnum.PLAYBACK: self.playback_action,
            StateEnum.CALIBRATION: self.slider_action,
            # StateEnum.CONVERSATION: ,
            StateEnum.KEY_CONTROL: self.keyboard_action,
            StateEnum.PHONE: Phone(),
        }
        self.state: StateEnum = StateEnum.IDLE
        # self.state: StateEnum = StateEnum.SENSOR

        self.timer = self.create_timer(1 / 30.0, self.update)

    def update(self):
        active_action = self.states.get(self.state, None)
        if active_action is not None:
            active_action.update()
            self.motor_id_pos = active_action.motor_id_pos
            print(self.motor_id_pos)
            self.write_motors()

    def callback_state(self, msg: String):
        requested_state = msg.data.upper()
        if requested_state in StateEnum.__members__:
            self.state = StateEnum[msg.data.upper()]
            active_action = self.states.get(self.state, None)
            if active_action is not None:
                print(f"{self.state=}, {active_action=}")
                active_action.setup()
        else:
            print(f"No existing {requested_state=}")

    @override
    def read_serial(self):
        while True:
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = BlsmRobotNode(
        name="robot_node",
        # motor_map=DEG2SERVO,
        motor_map=DEG2DXL,
        port=os.environ.get("BLSM_PORT", "/dev/ttyACM0"),
    )

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WebPageNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
