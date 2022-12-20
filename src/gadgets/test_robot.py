import unittest
from .robot import Robot

class RobotTestCase(unittest.TestCase):
    def setUp(self):
        self.robot = Robot()