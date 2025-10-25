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
from r0b0 import blsm_config
from scipy.spatial.transform import Rotation

import rclpy
import serial
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

from r0b0.config import (
    LOCALHOST,
    ROOT_DIR,
    SERVER_PORT,
    SOCKET_ADDR,
)
from r0b0_interfaces.msg import DeviceMotion, MotorCommands, MotorCommand

BLSM_PAGES_FOLDER = str(ROOT_DIR / "pages" / "blsm")


class RobotNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name)
        self.motor_id_pos: dict[str, float] = {}
        self.rotation = Rotation.from_euler("ZXY", angles=[0, 0, 0])

        # self.create_timer(
        #     0.1,
        #     callback=self.write_motors,
        # )

    @abstractmethod
    def write_motors(self): ...


class SerialRobotNode(RobotNode):
    def __init__(self, name, port="/dev/ttyACM0", baudrate=115200):
        super().__init__(name)
        self.serial = serial.Serial(port, baudrate)

    @override
    def write_motors(self):
        """Send the motor values as a string"""
        # params: str = "&".join(["=".join([str(k), str(v)])
        params: str = "&".join(
            [f"{k}={v:0.2f}" for k, v in self.motor_id_pos.items()]
        )
        params += "\n"
        self.serial.write(bytes(params, encoding="utf-8"))


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
    # class BlsmRobotNode(RobotNode):
    def __init__(self, motor_map=DEG2DXL, **kwargs):
        super().__init__(**kwargs)
        self._motor_map = motor_map
        self.device_motion_sub = self.create_subscription(
            DeviceMotion,
            "/blsm/device_motion",
            callback=self.ik,
            qos_profile=10,
        )
        self.key_event_sub = self.create_subscription(
            String,
            "/blsm/key_event",
            callback=self.update_rotation_from_keys,
            qos_profile=10,
        )
        self.slider_event_sub = self.create_subscription(
            MotorCommands,
            "/blsm/motor_cmd",
            callback=self.update_motors_from_sliders,
            qos_profile=10,
        )
        self.h: float = 0
        self.alpha: float = 0
        self.yaw_offset = 0
        self.mirror: bool = True
        self.sensitivity: float = 1
        self.breathe_rise_s: float = 4
        self.breathe_fall_s: float = 6
        self.breathe_amp_rad: float = 50
        # self.ts = time.time()
        self.breathing_thread = Thread(target=self.breathe, daemon=True)
        self.breathing_thread.start()

    def breathe(self):
        while True:
            t = time.time() % (self.breathe_rise_s + self.breathe_fall_s)

            if t < self.breathe_rise_s:
                mult = np.sin(2 * np.pi / (self.breathe_rise_s * 4) * t)
            else:
                mult = (
                    -np.sin(
                        2
                        * np.pi
                        / (self.breathe_fall_s * 4)
                        * (t - self.breathe_rise_s)
                    )
                    + 1
                )

            # self.h = 50 * np.sin(time.time()) + 50
            self.h = self.breathe_amp_rad * mult

            self._ik(
                self.rotation,
                alpha=self.alpha - self.yaw_offset,
                mirror=self.mirror,
                sensitivity=self.sensitivity,
            )
            time.sleep(0.05)

    def update_motors_from_sliders(self, msg: MotorCommands):
        for motor_cmd in msg.data:
            self.motor_id_pos.update({motor_cmd.name: motor_cmd.position_rad})
        # NOTE 250909: this is not dissimilar from rendering functions in e.g. neopixels
        print(self.motor_id_pos)
        self.write_motors()

    def update_rotation_from_keys(self, msg: String, delta_deg=10):
        # NOTE: these controls apply to the egocentric perspective,
        # i.e. from the robot's perspective
        delta_rad = delta_deg / 180 * np.pi
        # self.get_logger().info(f"{msg=}")
        match msg.data:
            case "KeyW":
                rotmat = [
                    [np.cos(-delta_rad), 0, -np.sin(-delta_rad)],
                    [0, 1, 0],
                    [np.sin(-delta_rad), 0, np.cos(-delta_rad)],
                ]
            #     rotvec = [0, -1, 0]
            case "KeyS":
                rotmat = [
                    [np.cos(delta_rad), 0, -np.sin(delta_rad)],
                    [0, 1, 0],
                    [np.sin(delta_rad), 0, np.cos(delta_rad)],
                ]

            #     rotvec = [0, 1, 0]
            case "KeyD":
                rotmat = [
                    [np.cos(-delta_rad), np.sin(-delta_rad), 0],
                    [-np.sin(-delta_rad), np.cos(-delta_rad), 0],
                    [0, 0, 1],
                ]
            #     rotvec = [0, 0, -1]
            case "KeyA":
                rotmat = [
                    [np.cos(delta_rad), np.sin(delta_rad), 0],
                    [-np.sin(delta_rad), np.cos(delta_rad), 0],
                    [0, 0, 1],
                ]
            #     rotvec = [0, 0, 1]
            case "KeyE":
                rotmat = [
                    [1, 0, 0],
                    [0, np.cos(-delta_rad), np.sin(-delta_rad)],
                    [0, -np.sin(-delta_rad), np.cos(-delta_rad)],
                ]
            #     rotvec = [-1, 0, 0]
            case "KeyQ":
                rotmat = [
                    [1, 0, 0],
                    [0, np.cos(delta_rad), np.sin(delta_rad)],
                    [0, -np.sin(delta_rad), np.cos(delta_rad)],
                ]
            #     rotvec = [1, 0, 0]
            case _:
                rotmat = np.eye(3)
            #     rotvec = [0,0,0]
        # self.rotation *= Rotation.from_rotvec(delta_rad*np.array(rotvec))
        self.rotation *= Rotation.from_matrix(rotmat)
        self._ik(
            self.rotation, alpha=self.rotation.as_euler("ZXY")[0], mirror=False
        )

    def ik(self, msg: DeviceMotion, sensitivity: float = 1.0):
        """
        Get position of the motors given orientation using inverse kinematics
        args:
            ori     orientation from sensors (Euler angles)
            accel   accelerometer readings
        returns:
            motor positions
        """
        # self.get_logger().info(f"{msg=}")
        # self.motor_id_pos.update(
        #     {1: int(np.interp(msg.xyz.x, [0, 6], [10, 170]))})
        # self.get_logger().info(f"{self.motor_id_pos=}")

        # yaw, pitch, roll
        # zyx / alpha,gamma,beta
        # print(ori)
        # [e_1, e_2, e_3, e_4, yaw_offset] = ori
        # [beta, gamma, alpha,_, yaw_offset] = ori
        # breakpoint()
        beta = msg.xyz.x
        gamma = msg.xyz.y
        self.alpha = msg.xyz.z
        portrait = msg.portrait

        self.yaw_offset = msg.yaw_offset

        angle_order = "ZXY"
        angles = [self.alpha, beta, gamma]

        r = Rotation.from_euler(angle_order, angles=angles)
        r = r * Rotation.from_euler("Z", np.pi / 2)
        if portrait:
            r = r * Rotation.from_euler("Y", np.pi / 2)
        self.rotation = r
        self.mirror = msg.mirror
        self.sensitivity = sensitivity
        # return self._ik(
        #     r,
        #     alpha=alpha - yaw_offset,
        #     mirror=msg.mirror,
        #     sensitivity=sensitivity,
        # )

    def _ik(self, r, alpha, mirror: bool, sensitivity: float = 1.0):
        # NOTE: could rewrite as matrix multiplication
        p_0 = [
            r.apply(_p)
            for _p in [blsm_config.p1_0, blsm_config.p2_0, blsm_config.p3_0]
        ]

        # calculate height
        # h = ((e_4 - 50) / 100.0) * h_range * h_fac + h_mid

        # init array for motor positions (tower_1-4)
        motor_pos = np.array([])
        # calculate distance motor must rotate
        for i, p_i in enumerate(p_0):
            # get just x,z components
            p_i_xz = p_i[[0, 2]]
            # p_i_xz = p_i
            # get displacement and its magnitude
            # del_h = p_i_xz-yaw.apply(p_list[i])
            del_h = p_i_xz - blsm_config.p_xz[i]
            # del_h = del_h[[0,2]]
            mag_del_h = np.linalg.norm(del_h)

            # calculate motor angle
            theta = np.rad2deg(
                mag_del_h / (blsm_config.r_w / sensitivity)
            ) * np.sign(del_h[1])

            # EXPERIMENTAL - only take the z-difference
            mag_del_h = del_h[1]
            theta = np.rad2deg(mag_del_h / (blsm_config.r_w / sensitivity))

            # print(theta)
            motor_pos = np.append(motor_pos, theta)

        # constrain tower motor range (50-130)
        motor_pos = np.maximum(
            np.minimum(motor_pos + self.h, blsm_config.h_max), blsm_config.h_min
        )

        # NOTE: not sure why this is here?
        # try:
        #     alpha -= yaw_offset
        # except:
        #     pass
        # alpha = r.as_euler("ZXY")[0]
        # alpha = r.as_rotvec()[2]
        # alpha -= yaw_offset

        # angle wrapping
        if alpha < -np.pi:
            alpha += 2 * np.pi
        elif alpha > np.pi:
            alpha -= 2 * np.pi

        # add the base motor for yaw (-140,140)
        motor_pos = np.append(
            motor_pos,
            np.maximum(
                np.minimum(np.rad2deg(blsm_config.base_mult * alpha), 150), -150
            ),
        )

        # 230620 - inverted since motors 2 and 3 swapped
        # in the refactor to r0b0
        motor_pos[3] *= -1

        if mirror:
            motor_pos[1], motor_pos[2] = motor_pos[2], motor_pos[1]
            motor_pos[3] *= -1

        self.motor_id_pos = {
            str(i + 1): int(np.interp(value, *self._motor_map[i]))
            for i, value in enumerate(motor_pos)
        }
        self.write_motors()


def main(args=None):
    rclpy.init(args=args)
    node = BlsmRobotNode("robot_node")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WebPageNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
