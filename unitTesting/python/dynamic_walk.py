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

from math import pi

from dynamic_graph.sot.core import FeaturePoint6d, FeatureGeneric, SOT
from dynamic_graph.sot.tools.se3 import SE3
from dynamic_graph.sot.dynamics.humanoid_robot import HumanoidRobot
from dynamic_graph.sot.dynamics.hrp2 import Hrp2

from dynamic_graph import enableTrace, plug

from dynamic_graph.sot.dynamics.tools import *

class HalfStep:
    startX = 0.
    startY = 0.
    # startZ is always 0.
    startTheta = 0.

    # finalX is always 0.
    finalY = 0.
    finalZ = 0.
    # finalTheta is always 0.

class DynamicWalking:
    leftFoot = 0
    rightFoot = 1

    footAltitude = 0.1

    robot = None
    supportFoot = None

    swingFoot = None

    def __init__(self, robot):
        self.robot = robot
        self.supportFoot = self.leftFoot

        self.swingFoot = HalfStep()
        self.swingFoot.startX = -0.1
        self.swingFoot.startY =  0.1
        self.swingFoot.startTheta = pi / 4.

        self.swingFoot.finalY = 0.1
        self.swingFoot.finalZ = 0.

    def computeFootPosition(op, swingFoot):
        # Support foot position in the waist frame.
        supportFootPos = self.robot.dynamicRobot.signal(op).value
        # Swing foot in the support foot: swingFoot

        # Swing foot in the waist frame.
        return SE3(supportFootPos) * SE3(swingFoot)

res = \
    SE3(robot.features['right-ankle'].reference.position.value) * \
    SE3((0., 0., 0., -0.1),
        (0., 0., 0.,  0.1),
        (0., 0., 0.,  0.),
        (0., 0., 0.,  1.))
print res


# Push tasks
#  Feet tasks.
solver.sot.push(robot.name + '.task.right-ankle')
solver.sot.push(robot.name + '.task.left-ankle')

#  Center of mass
# FIXME: trigger segv at exit.
solver.sot.push(robot.name + '.task.com')

dynamicWalking = DynamicWalking(robot)

for i in xrange(100):
    robot.device.increment(timeStep)

    if clt:
        clt.updateElementConfig(
            'hrp', robot.smallToFull(robot.device.state.value))
