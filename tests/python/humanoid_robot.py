#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

import unittest

from dynamic_graph.sot.dynamics.humanoid_robot import HumanoidRobot


class HumanoidRobotTest(unittest.TestCase):
    def test_simple(self):
        pass

    def test_model_not_exist(self):
        self.assertRaises(IOError, HumanoidRobot, "robot", True, "IDONOTEXIST")


if __name__ == '__main__':
    unittest.main()
