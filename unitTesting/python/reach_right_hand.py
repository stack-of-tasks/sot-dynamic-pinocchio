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

from dynamic_graph.sot.core import FeaturePoint6d, FeatureGeneric, SOT
from dynamic_graph.sot.dynamics.humanoid_robot import HumanoidRobot
from dynamic_graph.sot.dynamics.hrp2 import Hrp2

from dynamic_graph import enableTrace, plug

from tools import *


# Move right wrist
sdes = toList(robot.dynamicRobot.signal('right-wrist').value)
sdes[0][3] += 0.25 # Move reference point forward.
sdes[2][3] += 0.1 # Increment reference point altitude.
robot.features['right-wrist'].reference.signal('position').value = toTuple(sdes)
# Select translation only.
robot.features['right-wrist'].feature.signal('selec').value = '000111'
robot.tasks['right-wrist'].signal('controlGain').value = 1.

# Push tasks
#  Operational points tasks
solver.sot.push(robot.name + '.task.right-ankle')
solver.sot.push(robot.name + '.task.left-ankle')
solver.sot.push(robot.name + '.task.right-wrist')

#  Center of mass
solver.sot.push(robot.name + '.task.com')

# Main.
#  Main loop
for i in xrange(500):
    robot.robotSimu.increment(timeStep)

    if clt:
        clt.updateElementConfig(
            'hrp', robot.smallToFull(robot.robotSimu.signal('state').value))

print "FINISHED"
