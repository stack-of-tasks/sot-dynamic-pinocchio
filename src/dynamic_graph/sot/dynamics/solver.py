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

from dynamic_graph import plug
from dynamic_graph.sot.core import SOT

class Solver:
    robot = None
    sot = None

    def __init__(self, robot):
        self.robot = robot
        self.sot = SOT('solver')
        self.sot.signal('damping').value = 1e-6
        self.sot.setNumberDofs(self.robot.dimension)

        if robot.device:
            plug(self.sot.signal('control'), robot.device.signal('control'))
            plug(self.robot.device.state,
                 self.robot.dynamic.position)
    def push(self, taskName):
        """
        Proxy method to push a task in the sot
        """
        self.sot.push(taskName)
