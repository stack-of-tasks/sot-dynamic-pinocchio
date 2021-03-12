#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

import unittest

import numpy as np
from dynamic_graph.sot.dynamic_pinocchio import DynamicPinocchio
from dynamic_graph.sot.dynamic_pinocchio.humanoid_robot import AbstractHumanoidRobot

from example_robot_data import load_full


class Robot(AbstractHumanoidRobot):
    def __init__(self, name, urdfString=None, urdfFile=None):
        if urdfString is not None:
            self.loadModelFromString(urdfString)
        elif urdfFile is not None:
            self.loadModelFromUrdf(urdfFile)
        else:
            raise RuntimeError("You should provide either a URDF file or a URDF string")

        AbstractHumanoidRobot.__init__(self, name, None)

    def defineHalfSitting(self, q):
        pass


class HumanoidRobotTest(unittest.TestCase):
    def setUp(self):
        self.name = "talos"
        _, _, urdf, _ = load_full(self.name)
        self.urdf_file_name = urdf

    def test_non_instanciable_robot(self):
        class NonInstanciableRobot(AbstractHumanoidRobot):
            pass

        self.assertRaises(TypeError, NonInstanciableRobot, "non_instanciable_robot")

    def test_build_robot_from_string(self):

        with open(self.urdf_file_name, 'r') as urdf:
            urdfString = urdf.read()
        arobot = Robot("test_build_robot_from_string", urdfString=urdfString)

        # Test if the two vectors are identical:
        arobot.dynamic = DynamicPinocchio(self.name + "_dynamic")
        arobot.dynamic.setModel(arobot.pinocchioModel)
        arobot.dynamic.setData(arobot.pinocchioData)
        arobot.dynamic.add_signals()

        def get(s):
            s.recompute(0)
            return s.value

        loc_lowerJl = np.array(get(arobot.dynamic.lowerJl))
        pin_lowerJl = np.array(
            arobot.pinocchioModel.lowerPositionLimit[1:len(arobot.pinocchioModel.lowerPositionLimit)])

        for i in range(0, len(loc_lowerJl), 1):
            if not loc_lowerJl[i] == pin_lowerJl[i]:
                self.assertTrue(False, "lowerJl is not working")

    def test_build_robot_from_urdf(self):
        Robot("test_build_robot_from_string", urdfFile=self.urdf_file_name)


if __name__ == '__main__':
    unittest.main()
