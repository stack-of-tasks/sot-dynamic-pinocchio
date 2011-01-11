#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of dynamic-graph.
# dynamic-graph is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# dynamic-graph is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

import unittest

from dynamic_graph.sot.dynamics.humanoid_robot import HumanoidRobot

class HumanoidRobotTest(unittest.TestCase):
    def test_simple(self):
        pass

    def test_model_not_exist(self):
        self.assertRaises(IOError, HumanoidRobot, "robot", True, "IDONOTEXIST")



if __name__ == '__main__':
    unittest.main()
