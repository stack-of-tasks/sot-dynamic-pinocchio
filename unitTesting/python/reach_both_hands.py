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

import sys
from dynamic_graph.sot.dynamics.tools import *

# Move left wrist
reach(robot, 'left-wrist', 0.25, 0.25, 0.5)
reach(robot, 'right-wrist', 0.25, -0.25, 0.5)

# Push tasks
#  Operational points tasks
solver.sot.push(robot.tasks['right-ankle'].name)
solver.sot.push(robot.tasks['left-ankle'].name)
solver.sot.push(robot.tasks['right-wrist'].name)
solver.sot.push(robot.tasks['left-wrist'].name)

#  Center of mass
solver.sot.push(robot.comTask.name)

# Main.
#  Main loop
for i in xrange(500):
    robot.device.increment(timeStep)

    if clt:
        clt.updateElementConfig(
            'hrp', robot.smallToFull(robot.device.state.value))

finalPosition = (
    -0.0357296, -0.0024092699999999998, 0.033870600000000001,
     -0.00120006, 0.075338500000000003, 0.00028531699999999999,
     2.9108999999999999e-05, 0.0053813699999999999, -0.41716799999999998,
     0.52743700000000004, -0.185608, -0.0041716399999999999,
     2.9120099999999999e-05, 0.0053815199999999999, -0.41625699999999999,
     0.52549900000000005, -0.184581, -0.0041717899999999999, -0.00306069,
     -0.54035500000000003, 0.00036732799999999999, -0.028043100000000001,
     -0.69961799999999996, -0.84530499999999997, 0.11480700000000001,
     -0.61734999999999995, 0.088004899999999997, 1.0022899999999999,
     0.100354, -0.71066200000000002, 0.85328800000000005,
     -0.109959, -0.60156299999999996, -0.082422700000000002,
     1.0207200000000001, 0.10037500000000001)

checkFinalConfiguration(robot.device.state.value, finalPosition)
print "Exiting."
