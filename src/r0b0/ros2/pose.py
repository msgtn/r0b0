from dataclasses import dataclass

from scipy.spatial.transform import Rotation


@dataclass
class BlsmPose:
    h: float = 0
    rot: Rotation = Rotation.from_euler("ZXY", angles=[0, 0, 0])

    def reset(self):
        self.h = 0
        self.rot = Rotation.from_euler("ZXY", angles=[0, 0, 0])
