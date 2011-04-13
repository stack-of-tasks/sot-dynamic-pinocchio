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
reach(robot, 'left-wrist', 0.25, 0, 0.1)

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
    -0.015183500000000001, -0.00037148200000000002, -0.00065935600000000005,
     0.0137784, -0.022388600000000002, -0.036052000000000001,
     0.036032099999999997, -0.012359200000000001, -0.466526,
     0.87994899999999998, -0.39055299999999998, -0.00060408700000000001,
     0.036028499999999998, -0.0121996, -0.45257900000000001,
     0.86809899999999995, -0.39265, -0.00076379799999999999,
     -0.084612699999999999, -0.18716099999999999, 0.00038832500000000002,
     -0.0054475900000000004, 0.23852599999999999, -0.19830900000000001,
     0.17092299999999999, -0.48823699999999998, -0.014739800000000001,
     0.170987, 0.100064, -0.117492, 0.24870200000000001, 0.016264399999999998,
     -0.56795700000000005, 0.0040012399999999997, 0.18956200000000001, 0.100089)

checkFinalConfiguration(robot.device.state.value, finalPosition)
print "Exiting."
