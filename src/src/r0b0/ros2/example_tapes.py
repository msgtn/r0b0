"""Example tapes for BLSM robot playback demonstrations."""

import numpy as np
from scipy.spatial.transform import Rotation

from r0b0.ros2.pose import BlsmPose
from r0b0.ros2.tapes import Frame, Tape


def create_nod_tape() -> Tape:
    """Create a nodding animation (pitch forward and back)."""
    frames = []
    duration = 2.0  # seconds
    num_frames = 30

    for i in range(num_frames):
        t = i / (num_frames - 1) * duration
        # Sinusoidal nod: pitch forward then back
        pitch = 0.3 * np.sin(2 * np.pi * t / duration)  # radians
        rot = Rotation.from_euler("ZXY", [0, 0, pitch])
        frames.append(Frame(ts=t, pose=BlsmPose(h=50, rot=rot)))

    return Tape(name="nod", frames=frames)


def create_shake_tape() -> Tape:
    """Create a head shake animation (yaw left and right)."""
    frames = []
    duration = 2.0
    num_frames = 30

    for i in range(num_frames):
        t = i / (num_frames - 1) * duration
        # Shake side to side
        yaw = 0.4 * np.sin(2 * np.pi * t / duration)
        rot = Rotation.from_euler("ZXY", [yaw, 0, 0])
        frames.append(Frame(ts=t, pose=BlsmPose(h=50, rot=rot)))

    return Tape(name="shake", frames=frames)


def create_wave_tape() -> Tape:
    """Create a wave animation (height oscillation)."""
    frames = []
    duration = 3.0
    num_frames = 45

    for i in range(num_frames):
        t = i / (num_frames - 1) * duration
        # Wave up and down
        h = 50 + 40 * np.sin(2 * np.pi * t / duration)
        rot = Rotation.from_euler("ZXY", [0, 0, 0])
        frames.append(Frame(ts=t, pose=BlsmPose(h=h, rot=rot)))

    return Tape(name="wave", frames=frames)


def create_circle_tape() -> Tape:
    """Create a circular motion animation (roll in a circle)."""
    frames = []
    duration = 4.0
    num_frames = 60

    for i in range(num_frames):
        t = i / (num_frames - 1) * duration
        # Circular tilt motion
        angle = 2 * np.pi * t / duration
        pitch = 0.25 * np.sin(angle)
        roll = 0.25 * np.cos(angle)
        rot = Rotation.from_euler("ZXY", [0, pitch, roll])
        frames.append(Frame(ts=t, pose=BlsmPose(h=60, rot=rot)))

    return Tape(name="circle", frames=frames)


def create_breathe_tape() -> Tape:
    """Create a breathing animation (slow height rise and fall)."""
    frames = []
    rise_duration = 2.0
    fall_duration = 3.0
    duration = rise_duration + fall_duration
    num_frames = 50

    for i in range(num_frames):
        t = i / (num_frames - 1) * duration

        if t < rise_duration:
            # Rising phase
            mult = np.sin(2 * np.pi / (rise_duration * 4) * t)
        else:
            # Falling phase
            mult = (
                -np.sin(2 * np.pi / (fall_duration * 4) * (t - rise_duration))
                + 1
            )

        h = 80 * mult
        rot = Rotation.from_euler("ZXY", [0, 0, 0])
        frames.append(Frame(ts=t, pose=BlsmPose(h=h, rot=rot)))

    return Tape(name="breathe", frames=frames)


def create_look_around_tape() -> Tape:
    """Create a look around animation (yaw sweep with slight pitch)."""
    frames = []
    duration = 5.0
    num_frames = 75

    for i in range(num_frames):
        t = i / (num_frames - 1) * duration
        # Slow pan left, then right, then center
        progress = t / duration
        if progress < 0.25:
            # Look left
            yaw = -0.5 * (progress / 0.25)
        elif progress < 0.5:
            # Hold left, slight up
            yaw = -0.5
        elif progress < 0.75:
            # Sweep to right
            yaw = -0.5 + 1.0 * ((progress - 0.5) / 0.25)
        else:
            # Return to center
            yaw = 0.5 - 0.5 * ((progress - 0.75) / 0.25)

        pitch = 0.1 * np.sin(2 * np.pi * t / duration)
        rot = Rotation.from_euler("ZXY", [yaw, pitch, 0])
        frames.append(Frame(ts=t, pose=BlsmPose(h=55, rot=rot)))

    return Tape(name="look_around", frames=frames)


def get_example_tapes() -> list[Tape]:
    """Get all example tapes."""
    return [
        create_nod_tape(),
        create_shake_tape(),
        create_wave_tape(),
        create_circle_tape(),
        create_breathe_tape(),
        create_look_around_tape(),
    ]
