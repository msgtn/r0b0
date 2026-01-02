import time
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from r0b0.ros2.pose import BlsmPose


@dataclass
class Frame:
    ts: float
    pose: BlsmPose


@dataclass
class Tape:
    """Time-series list of poses for playback.

    Frames have timestamps relative to start of tape (first frame at ts=0).
    Supports linear interpolation between frames for smooth playback.
    """
    name: str
    frames: list[Frame] = field(default_factory=list)
    loop: bool = False
    ts_start: float | None = None

    @property
    def duration(self) -> float:
        """Total duration of the tape in seconds."""
        if not self.frames:
            return 0.0
        return self.frames[-1].ts - self.frames[0].ts

    @property
    def is_empty(self) -> bool:
        return len(self.frames) == 0

    def reset(self):
        """Reset playback state to beginning."""
        self.ts_start = None

    def get_progress(self) -> float:
        """Get current playback progress as a value between 0.0 and 1.0."""
        if self.ts_start is None or self.duration <= 0:
            return 0.0
        elapsed = time.time() - self.ts_start
        if self.loop and self.duration > 0:
            elapsed = elapsed % self.duration
        return min(1.0, max(0.0, elapsed / self.duration))

    def get_frame_at_ts(self, ts: float) -> BlsmPose | None:
        """Get interpolated pose at the given timestamp.

        Args:
            ts: Current time (absolute). On first call, this becomes the start time.

        Returns:
            Interpolated BlsmPose, or None if tape is empty or playback finished.
        """
        if self.is_empty:
            return None

        if self.ts_start is None:
            self.ts_start = ts

        # Calculate elapsed time since playback started
        elapsed = ts - self.ts_start

        # Handle looping
        if self.loop and self.duration > 0:
            elapsed = elapsed % self.duration

        # Check if playback finished (non-looping)
        if elapsed > self.duration and not self.loop:
            return self.frames[-1].pose

        # Find surrounding frames for interpolation
        tape_ts = self.frames[0].ts + elapsed

        # Binary search for efficiency with long tapes
        frame_timestamps = [f.ts for f in self.frames]
        idx = np.searchsorted(frame_timestamps, tape_ts, side='right') - 1
        idx = max(0, min(idx, len(self.frames) - 1))

        # If at or past last frame, return last frame
        if idx >= len(self.frames) - 1:
            return self.frames[-1].pose

        # Interpolate between frames[idx] and frames[idx+1]
        frame_a = self.frames[idx]
        frame_b = self.frames[idx + 1]

        # Calculate interpolation factor (0 to 1)
        dt = frame_b.ts - frame_a.ts
        if dt <= 0:
            return frame_a.pose

        t = (tape_ts - frame_a.ts) / dt
        t = max(0.0, min(1.0, t))

        # Interpolate height linearly
        h_interp = frame_a.pose.h + t * (frame_b.pose.h - frame_a.pose.h)

        # Interpolate rotation using SLERP
        slerp = Slerp([0, 1], Rotation.concatenate([frame_a.pose.rot, frame_b.pose.rot]))
        rot_interp = slerp([t])[0]

        return BlsmPose(h=h_interp, rot=rot_interp)
