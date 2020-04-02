#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

import unittest

from dynamic_graph.sot.dynamic_pinocchio.humanoid_robot import AbstractHumanoidRobot

class Robot (AbstractHumanoidRobot):
    def __init__ (self, name, urdfString=None, urdfFile=None):
        import pinocchio
        if urdfString is not None:
            self.loadModelFromString(urdfString)
        elif urdfFile is not None:
            self.loadModelFromUrdf(urdfFile)
        else:
            raise RuntimeError("You should provide either a URDF file or a URDF string")

        AbstractHumanoidRobot.__init__(self, name, None)

    def defineHalfSitting (self, q):
        pass


class HumanoidRobotTest(unittest.TestCase):
    def setUp(self):
        import os
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.r2d2_urdf_file = os.path.join(dir_path, "r2d2.urdf")

    def test_non_instanciable_robot(self):
        class NonInstanciableRobot (AbstractHumanoidRobot):
            pass
        self.assertRaises(TypeError, NonInstanciableRobot, "non_instanciable_robot")

    def test_build_robot_from_string(self):

        with open(self.r2d2_urdf_file, 'r') as urdf:
            urdfString = urdf.read()
        robot = Robot("test_build_robot_from_string", urdfString = urdfString)

    def test_build_robot_from_urdf(self):
        robot = Robot("test_build_robot_from_string", urdfFile = self.r2d2_urdf_file)


if __name__ == '__main__':
    unittest.main()
