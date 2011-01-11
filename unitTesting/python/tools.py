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

from optparse import OptionParser

from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.dynamics.hrp2 import Hrp2


# Robotviewer is optional
hasRobotViewer = True
try:
    import robotviewer
except ImportError:
    hasRobotViewer = False


####################
# Helper functions #
####################

def toList(tupleOfTuple):
    result = [[0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0]]
    for i in xrange(4):
        for j in xrange(4):
            result[i][j] = tupleOfTuple[i][j]
    return result

def toTuple(listOfList):
    return ((listOfList[0][0], listOfList[0][1],
             listOfList[0][2], listOfList[0][3]),

            (listOfList[1][0], listOfList[1][1],
             listOfList[1][2], listOfList[1][3]),

            (listOfList[2][0], listOfList[2][1],
             listOfList[2][2], listOfList[2][3]),

            (listOfList[3][0], listOfList[3][1],
             listOfList[3][2], listOfList[3][3]))

def displayHomogeneousMatrix(matrix):
    """
    Display nicely a 4x4 matrix (usually homogeneous matrix).
    """
    import itertools

    matrix_tuple = tuple(itertools.chain.from_iterable(matrix))

    formatStr = ''
    for i in xrange(4*4):
        formatStr += '{0[' + str(i) + ']: <10} '
        if i != 0 and (i + 1) % 4 == 0:
            formatStr += '\n'
    print formatStr.format(matrix_tuple)

def initRobotViewer():
    """Initialize robotviewer is possible."""
    clt = None
    if hasRobotViewer:
        try:
            clt = robotviewer.client()
        except:
            print "failed to connect to robotviewer"
    return clt


##################
# Helper classes #
##################

class Solver:
    robot = None
    sot = None

    def __init__(self, robot):
        self.robot = robot
        self.sot = SOT('solver')
        self.sot.signal('damping').value = 1e-6
        self.sot.setNumberDofs(self.robot.dimension)

        if robot.robotSimu:
            plug(self.sot.signal('control'), robot.robotSimu.signal('control'))
            plug(self.robot.robotSimu.signal('state'),
                 self.robot.dynamicRobot.signal('position'))


##################
# Initialization #
##################

# Parse options and enable robotviewer client if wanted.
clt = None
parser = OptionParser()
parser.add_option("-d", "--display",
                  action="store_true", dest="display", default=False,
                  help="enable display using robotviewer")
(options, args) = parser.parse_args()

if options.display:
    if not hasRobotViewer:
        print "Failed to import robotviewer client library."
    clt = initRobotViewer()
    if not clt:
        print "Failed to initialize robotviewer client library."


# Initialize the stack of tasks.
robot = Hrp2("robot", True)
timeStep = .02
solver = Solver(robot)
