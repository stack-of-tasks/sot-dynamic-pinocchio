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

from dynamic_graph.sot import SE3, R3, SO3
from dynamic_graph.sot.core import RobotSimu, FeaturePoint6dRelative, \
    FeatureGeneric, FeatureJointLimits, Task, Constraint, GainAdaptive, SOT
from dynamic_graph.sot.dynamics import AngleEstimator
from dynamic_graph import enableTrace, plug
from dynamic_graph.sot.se3 import R3, SO3, SE3

from dynamic_graph.sot.dynamics.dynamic_hrp2 import DynamicHrp2

from dynamic_graph.sot.dynamics.humanoid_robot import AbstractHumanoidRobot

class Hrp2(AbstractHumanoidRobot):
    """
    This class instanciates a simple humanoid robot with
    """

    halfSitting = (
        # Free flyer
        0., 0., 0., 0., 0. , 0.,

        # Legs
#        0., 0., -1.04720, 2.09440, -1.04720, 0.,
#        0., 0., -1.04720, 2.09440, -1.04720, 0.,

        0., 0., -0.453786, 0.872665, -0.418879, 0.,
        0., 0., -0.453786, 0.872665, -0.418879, 0.,

        # Chest and head
        0., 0., 0., 0.,

        # Arms
#        0.17453, -0.17453, -0.17453, -0.87266, 0., 0., 0.1,
#        0.17453,  0.17453,  0.17453, -0.87266, 0., 0., 0.1

        0.261799, -0.17453, 0., -0.523599, 0., 0., 0.1,
        0.261799, 0.17453,  0., -0.523599, 0., 0., 0.1,
        )


    def __init__(self, name, modelDir, xmlDir, simu):
        AbstractHumanoidRobot.__init__ (self, name, simu)

        modelName = 'HRP2JRLmainSmall.wrl'
        specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
        jointRankPath = xmlDir + '/HRP2LinkJointRankSmall.xml'

        self.dynamicRobot = DynamicHrp2(self.name + '.dynamics')
        self.dynamicRobot.setFiles(modelDir, modelName,
                                   specificitiesPath, jointRankPath)
        self.dynamicRobot.parse()
        self.dimension = self.dynamicRobot.getDimension()
        if self.dimension != len(self.halfSitting):
            raise "invalid half-sitting pose"

        self.forwardKinematics = DynamicHrp2(self.name + '.forwardKinematics')
        self.forwardKinematics.setFiles(modelDir, modelName,
                                        specificitiesPath, jointRankPath)
        self.forwardKinematics.parse()

        self.initializeRobot()
