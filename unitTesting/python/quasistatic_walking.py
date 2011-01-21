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


class QuasiStaticWalking:
    leftFoot = 0
    rightFoot = 1

    stateCoM_doubleToSingle = 0
    stateLifting = 1
    stateLanding = 2
    stateCoM_singleToDouble = 3

    time = {
        stateCoM_doubleToSingle : 1.,
        stateLifting :            5.,
        stateLanding :            5.,
        stateCoM_singleToDouble : 1.
        }

    footAltitude = 0.1

    robot = None
    supportFoot = None
    state = None
    nextStateSwitch = None
    initialFootPose = dict()
    t = None


    def __init__(self, robot):
        self.robot = robot
        self.supportFoot = self.leftFoot
        self.state = self.stateCoM_doubleToSingle
        self.nextStateSwitch = 0 # Next switch is now!

        self.initialFootPose['left-ankle'] = \
            self.robot.dynamic.signal('left-ankle').value
        self.initialFootPose['right-ankle'] = \
            self.robot.dynamic.signal('right-ankle').value

        self.t = None # Will be updated through the update method.

        self.robot.tasks['left-ankle'].controlGain.value = 1.
        self.robot.tasks['right-ankle'].controlGain.value = 1.


    # Move CoM to a particular operational point (usually left or right ankle).
    # op supports a special value called origin to move back to double support.
    def moveCoM(self, op):
        x = 0.
        y = 0.

        if op == 'origin':
            x = 0.
            y = 0.
        else:
            x = robot.dynamic.signal(op).value[0][3]
            y = robot.dynamic.signal(op).value[1][3]

        z = robot.featureComDes.errorIN.value[2]
        self.robot.featureComDes.errorIN.value = (x, y, z)

    def liftFoot(self, op):
        sdes = toList(robot.dynamic.signal(op).value)
        sdes[2][3] += self.footAltitude # Increment altitude.
        robot.features[op].reference.value = toTuple(sdes)

    def landFoot(self, op):
        robot.features[op].reference.value = \
            self.initialFootPose[op]

    def supportFootStr(self):
        if self.supportFoot == self.leftFoot:
            return 'left-ankle'
        else:
            return 'right-ankle'

    def flyingFootStr(self):
        if self.supportFoot == self.leftFoot:
            return 'right-ankle'
        else:
            return 'left-ankle'

    def do(self, state):
        if state == self.stateCoM_doubleToSingle:
            self.do_stateCoM_doubleToSingle()
        elif state == self.stateLifting:
            self.do_stateLifting()
        elif state == self.stateLanding:
            self.do_stateLanding()
        else:
            self.do_stateCoM_singleToDouble()


    def do_stateCoM_doubleToSingle(self):
        self.moveCoM(self.supportFootStr())
        self.nextStateSwitch = self.t + self.time[self.stateCoM_doubleToSingle]

    def do_stateLifting(self):
        self.liftFoot(self.flyingFootStr())
        self.nextStateSwitch = self.t + self.time[self.stateLifting]

    def do_stateLanding(self):
        self.landFoot(self.flyingFootStr())
        self.nextStateSwitch = self.t + self.time[self.stateLanding]

    def do_stateCoM_singleToDouble(self):
        self.moveCoM('origin')
        self.nextStateSwitch = self.t + self.time[self.stateCoM_singleToDouble]

        # Switch support foot.
        if self.supportFoot == self.leftFoot:
            self.supportFoot = self.rightFoot
        else:
            self.supportFoot = self.leftFoot


    def update(self, t):
        self.t = t

        # If step is finished.
        if self.t >= self.nextStateSwitch:
            # Change the current state.
            if self.state == self.stateCoM_singleToDouble:
                self.state = 0
            else:
                self.state += 1

            # Trigger actions to move to next state.
            self.do(self.state)


# Push tasks
#  Feet tasks.
solver.sot.push(robot.name + '.task.right-ankle')
solver.sot.push(robot.name + '.task.left-ankle')

#  Center of mass
# FIXME: trigger segv at exit.
solver.sot.push(robot.name + '.task.com')


# Main.

#  Parameters
steps = 3

#  Initialization
quasiStaticWalking = QuasiStaticWalking(robot)

#  Total time computation
stepTime = reduce(lambda acc, (u,v): acc + v,
                  quasiStaticWalking.time.iteritems(), 0.)
totalSteps = int((stepTime / timeStep) * steps)

#  Main loop
t = 0
for i in xrange(totalSteps + 1):
    t += timeStep
    robot.simu.increment(timeStep)

    quasiStaticWalking.update(t)

    if clt:
        clt.updateElementConfig(
            'hrp', robot.smallToFull(robot.simu.state.value))

#  Security: switch back to double support.
quasiStaticWalking.moveCoM('origin')
duration = quasiStaticWalking.time[quasiStaticWalking.stateCoM_singleToDouble]

for i in xrange(int(duration / timeStep)):
    t += timeStep
    robot.simu.increment(timeStep)

    if clt:
        clt.updateElementConfig(
            'hrp', robot.smallToFull(robot.simu.state.value))

finalPosition = (
    -0.0082169200000000008, -0.0126068, -0.00022860999999999999,
     0.019962199999999999, -0.0159528, -0.00037375100000000002,
     4.2206800000000002e-07, 0.0032752800000000002, -0.46070899999999998,
     0.88794600000000001, -0.41127999999999998, -0.023234500000000002,
     1.7543699999999999e-06, 0.00319733, -0.44445400000000002,
     0.85594800000000004, -0.39553700000000003, -0.0231565, 0.00126246,
     -0.0010669500000000001, -3.2293899999999999e-05, -0.00016289599999999999,
     0.26377099999999998, -0.178532, -0.00082797999999999997,
     -0.52297099999999996, -9.2273200000000003e-06, 0.000114984, 0.100008,
     0.26377400000000001, 0.171155, -0.00065098499999999998,
     -0.52324700000000002, -1.23291e-05, 6.0469500000000001e-05, 0.100009)

checkFinalConfiguration(robot.simu.state.value, finalPosition)
print "Exiting."
