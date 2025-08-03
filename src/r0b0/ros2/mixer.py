from r0b0_interfaces.msg import DeviceMotion
from std_msgs.msg import String
from rclpy import Node

class Mixer(Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)




class BlsmMixer(Node):
    def __init__(self, *args, **kwargs):

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