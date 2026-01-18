"""
On computer, go to https://localhost:8080/blsm_broadcast
On mobile, go to https://r0b0.ngrok.io/blsm_controller
"""

import json
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
    DEG2DXL,
    DEG2SERVO,
    Keyboard,
    Phone,
    Playback,
    Sensor,
    Slider,
)
from r0b0.ros2.example_tapes import get_example_tapes
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
        self.serial = None
        self.serial_connected = False
        self._port = port
        self._baudrate = baudrate
        self._connect_serial()

    def _connect_serial(self):
        """Attempt to connect to the serial port."""
        if self.serial_connected:
            return True
        try:
            self.serial = serial.Serial(self._port, self._baudrate)
            self.serial_connected = True
            self.get_logger().info(f"Serial connected on {self._port}")
            return True
        except serial.SerialException as e:
            self.serial = None
            self.serial_connected = False
            self.get_logger().warn(
                f"Serial port {self._port} not available: {e}. "
                "Running in disconnected mode."
            )
            return False

    @override
    def write_motors(self):
        """Send the motor values as a string"""
        if not self.serial_connected:
            return
        # params: str = "&".join(["=".join([str(k), str(v)])
        params: str = "&".join(
            [f"{k}={v:0.2f}" for k, v in self.motor_id_pos.items()]
        )
        params += "\n"
        try:
            self.serial.write(bytes(params, encoding="utf-8"))
            # print(f"write_motors {params}")
        except (serial.SerialException, OSError):
            self.get_logger().warn("Serial disconnected")
            self.serial_connected = False
            self.serial = None


class StateEnum(Enum):
    IDLE = 0
    SENSOR = 1
    PLAYBACK = 2
    CONVERSATION = 3
    KEY_CONTROL = 4
    PHONE = 5
    CALIBRATION = 6


class BlsmRobotNode(SerialRobotNode):
    def __init__(self, motor_map=DEG2DXL, **kwargs):
        super().__init__(**kwargs)
        self._motor_map = motor_map
        self.phone_action = Phone(motor_map=motor_map)
        self.device_motion_sub = self.create_subscription(
            DeviceMotion,
            "/blsm/device_motion",
            callback=self.phone_action.ik_from_msg,
            qos_profile=10,
        )
        self.keyboard_action = Keyboard(motor_map=motor_map)
        self.key_event_sub = self.create_subscription(
            String,
            "/blsm/key_event",
            callback=self.keyboard_action.update_rotation_from_keys,
            qos_profile=10,
        )
        self.slider_action = Slider(motor_map=motor_map)
        self.slider_event_sub = self.create_subscription(
            MotorCommands,
            "/blsm/motor_cmd",
            callback=self.slider_action.update_motors_from_sliders,
            qos_profile=10,
        )
        self.distance_pub = self.create_publisher(
            Int64, "/blsm/distance_sensor", qos_profile=10
        )
        self.playback_status_pub = self.create_publisher(
            String, "/blsm/playback/status", qos_profile=10
        )
        self.motor_pose_pub = self.create_publisher(
            String, "/blsm/motor_pose", qos_profile=10
        )
        self._pose_pub_counter = 0
        self.state_sub = self.create_subscription(
            String,
            "/blsm/state/request",
            callback=self.callback_state,
            qos_profile=10,
        )
        self.playback_action = Playback(
            status_pub=self.playback_status_pub, motor_map=motor_map
        )
        # Register example tapes for playback
        for tape in get_example_tapes():
            self.playback_action.register_tape(tape)
            self.get_logger().info(
                f"Registered tape: {tape.name} ({tape.duration:.1f}s)"
            )

        self.playback_sub = self.create_subscription(
            String,
            "/blsm/playback",
            callback=self.playback_action.playback_callback,
            qos_profile=10,
        )

        self.h: float = 0
        self.alpha: float = 0
        self.yaw_offset = 0
        self.mirror: bool = True
        self.sensitivity: float = 1

        self.sensor_action = (
            Sensor(
                serial=self.serial,
                distance_pub=self.distance_pub,
                motor_map=motor_map,
            )
            if self.serial_connected
            else None
        )

        self.states: dict[StateEnum, BlsmAction] = {
            StateEnum.IDLE: Breathe(motor_map=motor_map),
            StateEnum.PLAYBACK: self.playback_action,
            StateEnum.CALIBRATION: self.slider_action,
            # StateEnum.CONVERSATION: ,
            StateEnum.KEY_CONTROL: self.keyboard_action,
            StateEnum.PHONE: Phone(motor_map=motor_map),
        }
        if self.sensor_action is not None:
            self.states[StateEnum.SENSOR] = self.sensor_action
        self.state: StateEnum = StateEnum.IDLE

        self.timer = self.create_timer(1 / 30.0, self.update)

    def update(self):
        active_action = self.states.get(self.state, None)
        if active_action is not None:
            active_action.update()
            self.motor_id_pos = active_action.motor_id_pos
            # print(self.motor_id_pos)
            self.write_motors()
            # Publish pose at 15Hz (every 2nd frame of 30Hz update)
            self._pose_pub_counter += 1
            if self._pose_pub_counter >= 2:
                self._pose_pub_counter = 0
                pose = active_action.get_pose()
                self.motor_pose_pub.publish(String(data=json.dumps(pose)))

    def callback_state(self, msg: String):
        requested_state = msg.data.upper()
        if requested_state in StateEnum.__members__:
            new_state = StateEnum[msg.data.upper()]
            active_action = self.states.get(new_state, None)
            if active_action is not None:
                self.state = new_state
                self.get_logger().info(f"State changed to {self.state}")
                active_action.setup()
            else:
                self.get_logger().warn(
                    f"State {new_state} unavailable (serial not connected?)"
                )
        else:
            self.get_logger().warn(f"No existing state: {requested_state}")

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
