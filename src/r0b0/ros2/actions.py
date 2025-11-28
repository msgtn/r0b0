from dataclasses import dataclass
import time
import numpy as np
import py_trees
from scipy.spatial.transform import Rotation
import serial

from r0b0 import blsm_config
from r0b0.ros2.filters import ExponentialFilter
from std_msgs.msg import Int64

from r0b0_interfaces.msg import DeviceMotion, MotorCommands, MotorCommand

DIST2HEIGHT = [[0, 500], [0, 50]]


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


@dataclass
class BlsmPose:
    h: float = 0
    rot: Rotation = Rotation.from_euler("ZXY", angles=[0, 0, 0])


class BlsmAction(py_trees.behaviour.Behaviour):
    # NOTE: refactor with motor positions instead of pose
    def __init__(
        self,
        name: str = "breathe",
        motor_id_pos: dict | None = None,
        motor_map=DEG2SERVO,
    ):
        super().__init__(name)
        self.pose: BlsmPose = BlsmPose()
        if motor_id_pos is None:
            motor_id_pos = {}
        self.motor_id_pos: dict[str, float] = motor_id_pos
        self._motor_map = motor_map

    def setup(self, **kwargs): ...

    def initialise(self): ...

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None: ...

    def ik_from_rot(self, rot: Rotation, alpha, mirror: bool, sensitivity: float = 1.0):
        # NOTE: could rewrite as matrix multiplication
        p_0 = [
            rot.apply(_p)
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
            theta = np.rad2deg(mag_del_h / (blsm_config.r_w / sensitivity)) * np.sign(
                del_h[1]
            )

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


class Breathe(BlsmAction):
    def __init__(self, name: str = "breathe"):
        super().__init__(name)
        self.breathe_rise_s: float = 4
        self.breathe_fall_s: float = 6
        self.breathe_amp_rad: float = 50

    def setup(self, **kwargs):
        print("setup")

    def initialise(self):
        print("init")

    def update(self) -> py_trees.common.Status:
        t = time.time() % (self.breathe_rise_s + self.breathe_fall_s)

        if t < self.breathe_rise_s:
            mult = np.sin(2 * np.pi / (self.breathe_rise_s * 4) * t)
        else:
            mult = (
                -np.sin(
                    2 * np.pi / (self.breathe_fall_s * 4) * (t - self.breathe_rise_s)
                )
                + 1
            )

        self.pose.h = self.breathe_amp_rad * mult

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        print("terminate")


class Sensor(BlsmAction):
    def __init__(self, serial: serial.Serial, distance_pub, name: str = "breathe"):
        super().__init__(name)
        self.serial = serial
        self.distance_mm: int | None = None
        self.distance_filter = ExponentialFilter(alpha=0.5)
        self.distance_pub = distance_pub

    def setup(self, **kwargs): ...

    def initialise(self): ...

    def update(self) -> py_trees.common.Status:
        try:
            # TODO: handle jumps when sensor is covered,
            # from low values (~50) to max (~2500)
            # but to the sensors this looks the same
            # as quickly removing your hand from above the sensor
            # maybe expose the alpha parameter of the ExponentialFilter?
            # not sure if that's relevant to the curriculum
            # NOTE 251118 jumping may not be that bad,
            # could be the "cat" inching away until contact,
            # then purrs up
            data = self.serial.readline()
            if data:
                data = data.decode(errors="ignore").strip()
                data = int(data)
                self.distance_mm = self.distance_filter(data)
                self.distance_pub.publish(Int64(data=self.distance_mm))
                # TODO map this to some behavior,
                # i.e. map distance to the height,
                # and turn off the height adjustment in the breathing thread
                dist_within_range = (
                    DIST2HEIGHT[0][0] < self.distance_mm < DIST2HEIGHT[0][1]
                )
                self.breathing = not dist_within_range
                if dist_within_range:
                    self.pose.h = np.interp(
                        self.distance_mm, DIST2HEIGHT[0], DIST2HEIGHT[1]
                    )

                    self.ik_from_rot(
                        self.pose.rot,
                        alpha=0,
                        mirror=False,
                    )

        except serial.SerialException:
            ...
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None: ...


class Keyboard(BlsmAction):
    def update_rotation_from_keys(self, msg: String, delta_deg=10):
        # NOTE: these controls apply to the egocentric perspective,
        # i.e. from the robot's perspective
        delta_rad = delta_deg / 180 * np.pi
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
        self.pose.rot *= Rotation.from_matrix(rotmat)
        self.ik_from_rot(
            self.pose.rot, alpha=self.pose.rot.as_euler("ZXY")[0], mirror=False
        )


class Slider(BlsmAction):
    def update_motors_from_sliders(self, msg: MotorCommands):
        for motor_cmd in msg.data:
            self.motor_id_pos.update({motor_cmd.name: motor_cmd.position_rad})
        # NOTE 250909: this is not dissimilar from rendering functions in e.g. neopixels
        print(self.motor_id_pos)


class Phone(BlsmAction):
    def ik_from_msg(self, msg: DeviceMotion, sensitivity: float = 1.0):
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
        alpha = msg.xyz.z
        portrait = msg.portrait

        yaw_offset = msg.yaw_offset

        angle_order = "ZXY"
        angles = [alpha, beta, gamma]

        r = Rotation.from_euler(angle_order, angles=angles)
        r = r * Rotation.from_euler("Z", np.pi / 2)
        if portrait:
            r = r * Rotation.from_euler("Y", np.pi / 2)
        return self.ik_from_rot(
            r,
            alpha=alpha - yaw_offset,
            mirror=msg.mirror,
            sensitivity=sensitivity,
        )


def main():
    action = Breathe()
    try:
        while True:
            action.setup()
            for _unused_i in range(0, 12):
                action.tick_once()
                time.sleep(0.5)
            action.terminate(py_trees.common.Status.SUCCESS)
        print("\n")
    except KeyboardInterrupt:
        pass
    ...


if __name__ == "__main__":
    main()
