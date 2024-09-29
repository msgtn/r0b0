from threading import Thread
import time, datetime
import numpy as np
from functools import partial
from r0b0.utils.loaders import decode_msg, encode_msg

from .gadget import Gadget, Message
import logging

logging = logging.getLogger(__name__)
from dynamixel_python import DynamixelManager, DynamixelMotor, ReadError

BAUD_DICT = {57600: 1, 115200: 2, 1000000: 3, 9600: 0}
# for xl330-m{288,077}
# might be different for others
MODE_DICT = {
    "velocity": 1,
    "position": 3,
    "extended_position": 4,
}

T_LAST_CMD = 0
T_COOLDOWN = 300 / 10e3


class DynamixelRobot(Gadget, DynamixelManager):
    """
    A Gadget representing a Dynamixel-based robot.
    """

    @staticmethod
    def Message(*args, **kwargs):
        return Message(*args, **kwargs)

    def __init__(self, config, **kwargs):
        Gadget.__init__(self, config, request_timeout=0.1, **kwargs)
        DynamixelManager.__init__(
            self,
            usb_port=self.config.get("usb_port", "/dev/tty.usbserial-FT1SF1UM"),
            baud_rate=self.config.get("baud_rate", 57600),
        )
        # self.name = config['name']

        # add motors
        self.motors_by_id, self.motor_configs = {}, {}
        if self.config.get("motors", False):
            self.motors_by_id, self.motor_configs = self.add_motors_from_config(
                self.config["motors"]
            )

        # initialize
        self.power_up = self.init
        # NOTE - enabling and disabling is slow, use sparingly
        # self.enable, self.disable = self.enable_all, self.disable_all
        self.power_up()
        self.disable()
        # for motor_name,motor in self..items():
        for motor_name in self.motor_configs.keys():
            self.dxl_dict[motor_name].set_motor_config(self.motor_configs[motor_name])

        # breakpoint()
        self.enable()

        # set motor operating modes
        # this must be done after self.power_up()
        # so that the connections to the motors are established
        # for _,motor in self.motors_by_id.items():
        #     motor.init_mode()

        # define event handlers
        for _event in ["param", "position", "velocity"]:
            self.on(
                _event,
                handler=getattr(self, f"{_event}_event"),
                namespace=self.namespace,
            )
        self.on("enable", handler=self.enable, namespace=self.namespace)
        self.on("disable", handler=self.disable, namespace=self.namespace)

        self.moving_thread = Thread(
            target=self._moving_thread,
            # daemon=True,
        )
        self.POLL_MOVEMENT = False
        # self.motion_thread.start()
        logging.debug(f"namespace {self.namespace}")

        self.set_param = partial(self.access_param, rw_mode="set")
        self.get_param = partial(self.access_param, rw_mode="get")

    def enable(self, *args, **kwargs):
        # if not self.enabled:
        logging.debug("Enabling")
        self.POLL_MOVEMENT = False
        self.enabled = self.enable_all()

    def disable(self, *args, **kwargs):
        logging.debug("Disabling")
        self.POLL_MOVEMENT = True
        self.enabled = not self.disable_all()

    def add_motors_from_config(self, motor_config: list):
        """
        Add motors from a configuration dictionary.

        :param motor_config: A dictionary of motor configurations.
        :return: _description_
        """
        self.motors_by_id = {}
        motor_configs = {}
        self.motors_by_id.setdefault(None)
        for motor_config in self.config["motors"]:
            # cannot set operating mode here - must first init
            # if motor.get('mode',False):
            #     motor_mode = MODE_DICT.get(motor['mode'],None)
            #     if motor_mode is None:
            #         logging.warn(
            #             f'No mode {motor["mode"]} \
            #             for motor {motor["name"]}, \
            #             setting to "position."'
            #         )
            #     breakpoint()
            #     dxl_motor.set_operating_mode(motor_mode)

            motor = Motor.from_motor(
                self.add_motor(
                    dxl_name=motor_config["name"],
                    dxl_id=motor_config["id"],
                    dxl_model=motor_config["model"],
                )
            )

            self.motors_by_id.update({int(motor_config["id"]): motor})
            self.dxl_dict.update({motor_config["name"]: motor})
            motor_configs.update({motor_config["name"]: motor_config})

        return self.motors_by_id, motor_configs

    def add_motor(self, **motor_kwargs):
        return Motor.from_motor(self.add_dynamixel(**motor_kwargs), **motor_kwargs)

    def _moving_thread(self):
        """
        A thread that continuously polls the motors for movement.
        If the POLL_MOVEMENT flag is set to True, iterates through the motors,
        stores movement, position, and velocity information in a dictionary.
        If any motors are moving, emit an event.
        Currently set to emit velocity, but this should be refactored to either emit all relevant
        information or make selectable the parameter to emit.
        """
        # continuously check the
        # "is moving" flag and emit events
        while True:
            try:
                if self.POLL_MOVEMENT:
                    moving_dict = {}
                    position_dict = {}
                    velocity_dict = {}
                    for motor_name, motor in self.dxl_dict.items():
                        # If the torque is on, the motor is probably moving somewhere, so skip
                        if motor.get_torque_enable():
                            continue
                        # moving_dict.update({motor_name: motor.get_moving() * True})
                        # position_dict.update({motor_name: motor.get_present_position()})

                        # Calculate the velocity, which is reported on the range [0, 2**32]
                        present_velocity = motor.get_present_velocity()
                        velocity_direction = np.exp2(
                            np.round(np.log2(present_velocity + 1e-6) / 32.0) * 32
                        )
                        present_velocity -= velocity_direction
                        # velocity_dict.update({motor_name: present_velocity})
                        moving_dict.update(
                            {
                                motor_name: {
                                    "moving": motor.get_moving() * True,
                                    "position": motor.get_present_position() % 2**12,
                                    "velocity": present_velocity,
                                }
                            }
                        )
                    # logging.debug(moving_dict)
                    if any([v["moving"] for v in moving_dict.values()]):
                        # logging.debug("Moving")
                        # logging.debug(velocity_dict)
                        logging.debug("Moving")
                        # else:
                        # break
                        self.emit(
                            event="motor_motion",
                            # data=velocity_dict,
                            data=moving_dict,
                            namespace=self.namespace,
                        )
                    time.sleep(50e-3)
            except:
                pass

    def access_param(self, param, motor_id_kwargs, rw_mode="set"):
        return_dict = {m_id: {} for m_id in motor_id_kwargs.keys()}
        for motor_id, motor_kwargs in motor_id_kwargs.items():
            # _motor = self.dxl_dict.get(str(motor_id),None)
            _motor = self.motors_by_id.get(motor_id, None)

            # TODO - packet should just be in a dict
            # with 'data' as the default arg
            # ps2014.set_param('goal_velocity',{1:{'data':-1600}})
            if _motor is not None:
                return_dict.update(
                    {motor_id: getattr(_motor, f"{rw_mode}_{param}")(**motor_kwargs)}
                )
        logging.debug(f"{datetime.datetime.now()}, {param}, {motor_id_kwargs}")
        return return_dict

    # def turn_to_list(self):
    def _var2list(self, *_vars):
        _return_list = []
        for _var in _vars:
            if not isinstance(_var, list):
                _return_list.append([_var])
            else:
                _return_list.append(_var)
        return _return_list

    def _msg2kwargs(self, msg):
        if getattr(msg, "motor_kwargs", None) is None:
            msg.__dict__.update({"motor_kwargs": msg.value})
        [motor_ids, motor_kwargs] = self._var2list(msg.motor_id, msg.motor_kwargs)
        motor_kwargs = [{"data": mk} for mk in motor_kwargs]
        return {m_id: m_kwargs for m_id, m_kwargs in zip(motor_ids, motor_kwargs)}

    @decode_msg
    def position_event(self, data):
        # msg is a data object
        # msg = data["msg"]
        # msg = data.get("msg", data)
        if "msg" in data:
            msg = data["msg"]
        else:
            msg = Message(**data)
        logging.debug(msg.value)
        logging.debug(f"MSG2KWARGS {datetime.datetime.now()}")
        motor_id_kwargs = self._msg2kwargs(msg)
        # logging.debug("motorkwargs")
        # logging.debug(motor_id_kwargs)

        # TODO - maybe calcualte this before packing with self._msg2kwargs
        # that might be cleaner
        if not getattr(msg, "absolute"):
            #     # handle calculation of relative position
            #     # get current position
            # logging.debug('relative')
            present_positions = self.get_param(
                "present_position", {m_id: {} for m_id, _ in motor_id_kwargs.items()}
            )
            # Sometimes results in very large number,
            # That just needs to be modulo'd by 2**12
            for pos in present_positions.keys():
                present_positions[pos] %= 2**12
            logging.debug(present_positions)
            relative_positions = [m_v["data"] for m_v in motor_id_kwargs.values()]
            motor_ids = list(motor_id_kwargs.keys())
            # motor_id_kwargs.update({m_id:})
            # logging.debug(present_positions)
            motor_id_kwargs.update(
                {
                    m_id: {"data": rel_pos + pres_pos}
                    for m_id, pres_pos, rel_pos in zip(
                        motor_ids, present_positions.values(), relative_positions
                    )
                }
            )
            #     # calculate differences
            #     # update values
            pass
        logging.debug(motor_id_kwargs)
        logging.debug(f"ENABLE {datetime.datetime.now()}")
        # self.enable()
        logging.debug(f"POSITION_EVENT {datetime.datetime.now()}")
        self.set_param(
            "goal_position",
            motor_id_kwargs,
        )
        self.emit(event="position", data=motor_id_kwargs, namespace=self.namespace)

    @decode_msg
    def velocity_event(self, data):
        # msg is a data object
        msg = data["msg"]
        motor_id_kwargs = self._msg2kwargs(msg)

        if not getattr(msg, "absolute") == "absolute":
            # handle calculation of relative velocity
            velocity_present = self.get_param(
                "present_velocity", {m_id: {} for m_id in motor_id_kwargs.keys()}
            )

            # get current position
            # calculate differences
            # update values
            pass
        self.enable()
        self.set_param(
            "goal_velocity",
            motor_id_kwargs,
            # {m_id:{'data':m_k} for m_id,m_k in motor_id_kwargs},
        )

    @decode_msg
    def param_event(self, data):
        msg = data["msg"]
        if not isinstance(msg.motor_id, list):
            msg.motor_id = [msg.motor_id]
            msg.value = [msg.value]

    @decode_msg
    def _position_event(self, data):
        """
        Deprecated.

        :param data: _description_
        """
        msg = data["msg"]
        if not isinstance(msg.motor_id, list):
            msg.motor_id = [msg.motor_id]
            msg.value = [msg.value]
        for motor_id, motor_value in zip(msg.motor_id, msg.value):
            motor = self.motors_by_id.get(motor_id, None)
            if motor is None:
                logging.debug(f"No motor ID {motor_id} found, skipping")
                continue
            motor.t_last_cmd = time.time()

            motor.set_profile_velocity(16000)
            motor.set_goal_position(int(motor_value))
            motor.believed_position = int(motor_value)

    def _deg2dxl(self, deg, deg_range=[-150, 150], dxl_range=[0, 4096]):
        return int(np.interp(deg, deg_range, dxl_range))


class Motor(DynamixelMotor):
    def __init__(self, **kwargs):
        # super().__init__(self,**kwargs)
        self.__dict__.update(**kwargs)
        self.mode = kwargs.get("mode", "position")
        self.t_last_cmd = 0

        self.enable = partial(self.set_param, param="torque_enable", value=True)
        self.disable = partial(self.set_param, param="torque_enable", value=False)

        # lookup the operating mode IDs
        # self.mode = MODE_DICT[getattr(self,'mode','position')]
        # self.init_mode = partial(self.set_mode,
        # mode=self.mode)

    @classmethod
    def from_motor(cls, dxl_motor: DynamixelMotor, **kwargs):
        return cls(**dxl_motor.__dict__, **kwargs)

    def set_param(self, param: str, value):
        self.access_param(param, value, rw_mode="set")

    def get_param(self, param: str, value=None):
        self.access_param(param, rw_mode="get")

    def access_param(self, param: str, value=None, rw_mode="set"):
        if rw_mode == "get":
            value = None
        if getattr(self, f"{rw_mode}_{param}", False):
            getattr(self, f"{rw_mode}_{param}")(value)
        else:
            logging.warning(f"Motor {self.name} could not {rw_mode} parameter {param}")

    def set_mode(self, mode):
        self.disable()
        self.set_param("operating_mode", mode)
        self.enable()

    def calibrate_homing_offset(self):
        self.disable()
        self.set_param("homing_offset", -(self.get_present_position() % 2**12))

    def set_motor_config(self, config: dict):
        # pass
        for param, data in config.items():
            self.set_param(param, data)
