from dataclasses import dataclass

from r0b0.ros2.pose import BlsmPose


@dataclass
class Frame:
    ts: float
    pose: BlsmPose


# NOTE: cannot assume static framerate
@dataclass
class Tape:
    name: str
    frames: list[Frame]
    frame_idx: int = 0
    ts_start: float | None = None

    def reset(self):
        self.frame_idx = 0
        self.ts_start = None

    def get_frame_at_ts(self, ts: float):
        if self.ts_start is None:
            self.ts_start = ts

        # NOTE: lineraly interpolate between frames

        # ret = self.frames[self.frame_idx]
        # if self.frame_idx < len(self.frames) - 1:
        #     self.frame_idx += 1
        return ret
