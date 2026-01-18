from dataclasses import dataclass
import termios
import time
import numpy as np
import py_trees
from scipy.spatial.transform import Rotation
import serial

from r0b0 import blsm_config
from r0b0.ros2.filters import ExponentialFilter
from r0b0.ros2.pose import BlsmPose
from r0b0.ros2.tapes import Tape
from std_msgs.msg import Int64, String

from r0b0_interfaces.msg import DeviceMotion, MotorCommands, MotorCommand

DIST2HEIGHT = [[0, 500], [0, 80]]


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


class BlsmAction(py_trees.behaviour.Behaviour):
    # NOTE: refactor with motor positions instead of pose
    def __init__(
        self,
        name: str = "breathe",
        motor_id_pos: dict | None = None,
        motor_map=DEG2DXL,
    ):
        super().__init__(name)
        self.pose: BlsmPose = BlsmPose()
        if motor_id_pos is None:
            motor_id_pos = {}
        self.motor_id_pos: dict[str, float] = motor_id_pos
        self._motor_map = motor_map

    def setup(self, **kwargs):
        print("setup")
        self.pose.reset()

    def initialise(self):
        print("init")

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None: ...

    def get_pose(self) -> dict:
        """Return current pose as a dict for visualization."""
        euler = self.pose.rot.as_euler("ZXY")
        return {
            "h": float(self.pose.h),
            "yaw": float(euler[0]),
            "pitch": float(euler[1]),
            "roll": float(euler[2]),
        }

    def ik_from_rot(
        self, rot: Rotation, alpha, mirror: bool, sensitivity: float = 1.0
    ):
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
            np.minimum(motor_pos + self.pose.h, blsm_config.h_max),
            blsm_config.h_min,
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
    def __init__(self, name: str = "breathe", motor_map=DEG2DXL):
        super().__init__(name, motor_map=motor_map)
        self.breathe_rise_s: float = 4
        self.breathe_fall_s: float = 6
        self.breathe_amp_rad: float = 50

    def update(self) -> py_trees.common.Status:
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

        self.pose.h = self.breathe_amp_rad * mult
        print(self.pose.h)
        self.ik_from_rot(self.pose.rot, alpha=0, mirror=False)

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        print("terminate")


class Sensor(BlsmAction):
    def __init__(
        self,
        serial: serial.Serial,
        distance_pub,
        name: str = "sensor",
        motor_map=DEG2DXL,
    ):
        super().__init__(name, motor_map=motor_map)
        self.serial = serial
        self.distance_mm: int | None = None
        self.distance_filter = ExponentialFilter(alpha=0.5)
        self.distance_pub = distance_pub
        self._last_publish_time = 0  # Add this for rate limiting
        self._publish_interval = 0.1  # seconds (10 Hz)
        # self._publish_interval = 1.0  # seconds (10 Hz)
        self._active = False

    def setup(self, **kwargs):
        super().setup(**kwargs)
        # Flush stale serial data when entering Sensor mode
        try:
            if self.serial.is_open:
                self.serial.reset_input_buffer()
        except (serial.SerialException, OSError, termios.error):
            pass
        self._active = True

    def terminate(self, new_status):
        self._active = False

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
            if not self.serial.is_open or self.serial.in_waiting == 0:
                return py_trees.common.Status.RUNNING
            data = self.serial.readline()
            if data:
                data = data.decode(errors="ignore").strip()
                try:
                    data = int(data)
                    self.distance_mm = int(self.distance_filter(data))
                    now = time.time()
                    if now - self._last_publish_time > self._publish_interval:
                        self.distance_pub.publish(Int64(data=self.distance_mm))
                        self._last_publish_time = now
                    # TODO map this to some behavior,
                    # i.e. map distance to the height,
                    # and turn off the height adjustment in the breathing thread
                    # print(f"{self.distance_mm=}")
                    print(data)
                    dist_within_range = (
                        DIST2HEIGHT[0][0] < self.distance_mm < DIST2HEIGHT[0][1]
                    )
                    if dist_within_range:
                        self.pose.h = np.interp(
                            self.distance_mm, DIST2HEIGHT[0], DIST2HEIGHT[1]
                        )
                    else:
                        self.pose.h = DIST2HEIGHT[1][1]

                    self.ik_from_rot(
                        self.pose.rot,
                        alpha=0,
                        mirror=False,
                    )
                except:
                    print(f"error parsing {data=}")
                    pass

        except (serial.SerialException, OSError, termios.error):
            # Device disconnected or I/O error
            pass
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
        # self.pose.rot *= Rotation.from_matrix(rotmat)
        self.pose.rot = Rotation.from_matrix(rotmat) * self.pose.rot
        # Clamp
        # TODO: there is a bug causing yaw at the ends to start pitching,
        # results in rotation in bad state
        euler = self.pose.rot.as_euler("ZXY")  # [yaw, pitch, roll]
        clamp = False
        if clamp:
            max_yaw = np.pi / 2.1
            max_angle = np.pi / 4
            euler[0] = np.clip(euler[0], -max_yaw, max_yaw)  # yaw
            euler[1] = np.clip(euler[1], -max_angle, max_angle)  # pitch
            euler[2] = np.clip(euler[2], -max_angle, max_angle)  # roll
            self.pose.rot = Rotation.from_euler("ZXY", euler)
        self.ik_from_rot(self.pose.rot, alpha=euler[0], mirror=False)


class Slider(BlsmAction):
    # def __init__(
    #     self,
    #     motor_map=DEG2DXL,
    # ):
    #     super().__init__(name, motor_map=motor_map)

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


class Playback(BlsmAction):
    """Playback recorded Tapes (time-series lists of poses).

    Tapes can be loaded via `load_tape()` or the ROS2 callback `playback_callback()`.
    Supports play, pause, stop, and loop functionality.
    """

    def __init__(self, status_pub=None, motor_map=DEG2DXL, *args, **kwargs):
        super().__init__(*args, motor_map=motor_map, **kwargs)
        self.tape_loaded: Tape | None = None
        self.tapes: dict[str, Tape] = {}
        self._playing: bool = False
        self._loop: bool = False
        self._status_pub = status_pub
        self._last_status_time: float = 0
        self._status_interval: float = 1 / 15.0  # 15 Hz status updates

    def setup(self, **kwargs):
        self.tape_loaded = None
        self._playing = False
        self.pose.reset()
        self._publish_status("stopped")

    def initialise(self):
        # Reset tape to beginning when behavior becomes active
        if self.tape_loaded is not None:
            self.tape_loaded.reset()

    def _publish_status(self, state: str | None = None):
        """Publish playback status to ROS2 topic."""
        if self._status_pub is None:
            return

        import json

        # Determine state if not provided
        if state is None:
            if self._playing:
                state = "playing"
            else:
                state = "stopped"

        status = {
            "state": state,
            "tape_name": self.tape_loaded.name if self.tape_loaded else "",
            "progress": 0.0,
            "current_time": 0.0,
            "total_time": 0.0,
        }

        # Add progress info if tape is loaded
        if self.tape_loaded is not None:
            progress = self.tape_loaded.get_progress()
            status["progress"] = progress
            status["current_time"] = progress * self.tape_loaded.duration
            status["total_time"] = self.tape_loaded.duration

            # Add current pose info
            euler = self.pose.rot.as_euler("ZXY")
            status["pose"] = {
                "h": float(self.pose.h),
                "yaw": float(euler[0]),
                "pitch": float(euler[1]),
                "roll": float(euler[2]),
            }

        self._status_pub.publish(String(data=json.dumps(status)))

    def update(self) -> py_trees.common.Status:
        if self._playing and self.tape_loaded is not None:
            pose = self.tape_loaded.get_frame_at_ts(time.time())
            if pose is not None:
                self.pose = pose
                self.ik_from_rot(self.pose.rot, alpha=0, mirror=False)

                # Publish status at regular intervals
                now = time.time()
                if now - self._last_status_time >= self._status_interval:
                    self._publish_status("playing")
                    self._last_status_time = now
            elif not self._loop:
                # Tape finished and not looping
                self._playing = False
                self._publish_status("stopped")
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self._playing = False
        self._publish_status("stopped")

    def register_tape(self, tape: Tape):
        """Register a tape for playback by name."""
        self.tapes[tape.name] = tape

    def load_tape(self, name: str, loop: bool = False) -> bool:
        """Load a tape by name for playback.

        Args:
            name: Name of the tape to load.
            loop: Whether to loop playback.

        Returns:
            True if tape was found and loaded, False otherwise.
        """
        tape = self.tapes.get(name)
        if tape is None:
            return False
        self.tape_loaded = tape
        self.tape_loaded.loop = loop
        self._loop = loop
        return True

    def play(self):
        """Start or resume playback."""
        if self.tape_loaded is not None:
            if not self._playing:
                # Reset tape timing when starting fresh
                self.tape_loaded.reset()
            self._playing = True
            self._publish_status("playing")

    def pause(self):
        """Pause playback (can be resumed with play())."""
        self._playing = False
        self._publish_status("paused")

    def stop(self):
        """Stop playback and reset to beginning."""
        self._playing = False
        if self.tape_loaded is not None:
            self.tape_loaded.reset()
        self.pose.reset()
        self._publish_status("stopped")

    def playback_callback(self, msg: String):
        """ROS2 callback to handle playback commands.

        Accepts JSON commands: {"action": "play", "tape_name": "...", "loop": bool}
        Or simple strings: tape name to play, "stop" to stop, "pause" to pause.
        """
        import json

        data = msg.data.strip()

        # Try parsing as JSON first
        try:
            cmd = json.loads(data)
            action = cmd.get("action", "")

            if action == "play":
                tape_name = cmd.get("tape_name", "")
                loop = cmd.get("loop", False)
                if tape_name and self.load_tape(tape_name, loop=loop):
                    self.play()
                else:
                    print(f"No tape '{tape_name}' registered")
            elif action == "pause":
                self.pause()
            elif action == "stop":
                self.stop()
            return
        except json.JSONDecodeError:
            pass

        # Fall back to simple string commands
        if data == "stop":
            self.stop()
        elif data == "pause":
            self.pause()
        elif self.load_tape(data, loop=self._loop):
            self.play()
        else:
            print(f"No tape '{data}' registered")


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
