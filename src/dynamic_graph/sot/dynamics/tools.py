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

def displayHrp2Configuration(cfg):
    if len(cfg) != 36:
        raise "bad configuration size"
    str = ''
    str += 'Free flyer:\n'
    str += ' translation {0[0]: <+10f} {0[1]: <+10f} {0[2]: <+10f}\n'
    str += ' rotation    {0[3]: <+10f} {0[4]: <+10f} {0[5]: <+10f}\n'
    str += 'Left leg:\n'
    str += ' hip         {0[6]: <+10f} {0[7]: <+10f} {0[8]: <+10f}\n'
    str += ' knee        {0[9]: <+10f}\n'
    str += ' ankle       {0[10]: <+10f} {0[11]: <+10f}\n'
    str += 'Right leg:\n'
    str += ' hip         {0[12]: <+10f} {0[13]: <+10f} {0[14]: <+10f}\n'
    str += ' knee        {0[15]: <+10f}\n'
    str += ' ankle       {0[16]: <+10f} {0[17]: <+10f}\n'
    str += 'Chest:       {0[18]: <+10f} {0[19]: <+10f}\n'
    str += 'Head:        {0[20]: <+10f} {0[21]: <+10f}\n'
    str += 'Left arm:\n'
    str += ' shoulder    {0[22]: <+10f} {0[23]: <+10f} {0[24]: <+10f}\n'
    str += ' elbow       {0[25]: <+10f}\n'
    str += ' wrist       {0[26]: <+10f} {0[27]: <+10f}\n'
    str += ' clench      {0[28]: <+10f}\n'
    str += 'Left arm:\n'
    str += ' shoulder    {0[29]: <+10f} {0[30]: <+10f} {0[31]: <+10f}\n'
    str += ' elbow       {0[32]: <+10f}\n'
    str += ' wrist       {0[33]: <+10f} {0[34]: <+10f}\n'
    str += ' clench      {0[35]: <+10f}\n'
    print str.format(cfg)


def initRobotViewer():
    """Initialize robotviewer is possible."""
    clt = None
    if hasRobotViewer:
        try:
            clt = robotviewer.client()
        except:
            print "failed to connect to robotviewer"
    return clt

def reach(robot, op, tx, ty, tz):
    sdes = toList(robot.dynamic.signal(op).value)
    sdes[0][3] += tx
    sdes[1][3] += ty
    sdes[2][3] += tz
    robot.features[op].reference.value = toTuple(sdes)
    # Select translation only.
    robot.features[op]._feature.signal('selec').value = '000111'
    robot.tasks[op].signal('controlGain').value = 1.

def sqrDist(value, expectedValue):
    """Compute the square of the distance between two configurations."""
    return reduce(lambda acc, (a, b): acc + abs(a - b) * abs(a - b),
                  zip(value, expectedValue), 0.)

def checkFinalConfiguration(position, finalPosition):
    if sqrDist(position, finalPosition) >= 1e-3:
        print "Wrong final position. Failing."
        print "Value:"
        displayHrp2Configuration(position)
        print "Expected value:"
        displayHrp2Configuration(finalPosition)
        print "Difference:"
        displayHrp2Configuration(map(lambda (x, y): x - y,
                                     zip(position, finalPosition)))
        sys.exit(1)

##################
# Initialization #
##################

import sys
if 'argv' in sys.__dict__:
    from optparse import OptionParser
    from dynamic_graph.sot.dynamics.solver import Solver

    # Parse options and enable robotviewer client if wanted.
    clt = None
    parser = OptionParser()
    parser.add_option("-d", "--display",
                      action="store_true", dest="display", default=False,
                      help="enable display using robotviewer")
    parser.add_option("-r", "--robot",
                      action="store", dest="robot", default="Hrp2Laas",
                      help="Specify which robot model to use")
    (options, args) = parser.parse_args()

    if options.display:
        if not hasRobotViewer:
            print "Failed to import robotviewer client library."
        clt = initRobotViewer()
        if not clt:
            print "Failed to initialize robotviewer client library."


    # Initialize the stack of tasks.
    robots = {
        "Hrp2Laas" : Hrp2Laas,
        "Hrp2Jrl"  : Hrp2Jrl
        }
    if not options.robot in robots:
        raise RuntimeError (
            "invalid robot name (should by Hrp2Laas or Hrp2Jrl)")
    robot = robots[options.robot]("robot")
    timeStep = .005
    solver = Solver(robot)
