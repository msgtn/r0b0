import unittest
from .dxl_robot import Robot


class RobotTestCase(unittest.TestCase):
    def setUp(self):
        self.robot = Robot()
