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

from dynamic_graph.sot.dynamics.tools import *

from dynamic_graph.sot.dynamics.feet_follower import FeetFollowerFromFile
feetFollower = FeetFollowerFromFile('feet-follower')

feetFollower.feetToAnkleLeft = robot.dynamic.getAnklePositionInFootFrame()
feetFollower.feetToAnkleRight = robot.dynamic.getAnklePositionInFootFrame()

plug(feetFollower.signal('com'), robot.featureComDes.signal('errorIN'))
plug(feetFollower.signal('left-ankle'),
     robot.features['left-ankle'].reference)
plug(feetFollower.signal('right-ankle'),
     robot.features['right-ankle'].reference)

robot.comTask.signal('controlGain').value = 50.
robot.tasks['left-ankle'].signal('controlGain').value = 50.
robot.tasks['right-ankle'].signal('controlGain').value = 50.

# Push tasks
#  Operational points tasks
solver.sot.push(robot.name + '.task.right-ankle')
solver.sot.push(robot.name + '.task.left-ankle')

#  Center of mass
solver.sot.push(robot.name + '.task.com')

# Main.
#  Main loop
for i in xrange(500):
    robot.device.increment(timeStep)

    if clt:
        clt.updateElementConfig(
            'hrp', robot.smallToFull(robot.device.state.value))


finalPosition = (
    -0.015361, -0.0049075500000000001, -0.00047065200000000001, -0.0172946,
     -0.020661800000000001, 0.0374547, -0.037641599999999997,
     0.025434399999999999, -0.45398100000000002, 0.86741800000000002,
     -0.39213799999999999, -0.0089269499999999995, -0.037646100000000002,
     0.025648199999999999, -0.46715499999999999, 0.87717599999999996,
     -0.38872200000000001, -0.0091408199999999992, 0.080488199999999996,
     -0.18355399999999999, -0.00036695100000000002, -0.0056776600000000002,
     -0.12173299999999999, -0.23972599999999999, -0.00637303,
     -0.56908000000000003, 0.00296262, 0.19108900000000001, 0.100088,
     0.23896800000000001, 0.21485599999999999, -0.18973400000000001,
     -0.49457699999999999, 0.040646799999999997, 0.16970299999999999, 0.100067)

checkFinalConfiguration(robot.device.state.value, finalPosition)
print "Exiting."
