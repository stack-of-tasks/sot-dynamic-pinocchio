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
from dynamic_graph.sot.core.feature_position import FeaturePosition
from dynamic_graph.sot.core import RobotSimu, FeaturePoint6dRelative, \
    FeatureGeneric, FeatureJointLimits, Task, Constraint, GainAdaptive, SOT

from dynamic_graph.sot.dynamics.parser import Parser
from dynamic_graph.sot.dynamics import AngleEstimator

from dynamic_graph import plug

I3 = reduce(lambda m, i: m + (i*(0.,)+(1.,)+ (2-i)*(0.,),), range(3), ())
I4 = reduce(lambda m, i: m + (i*(0.,)+(1.,)+ (2-i)*(0.,),), range(4), ())

class AbstractHumanoidRobot (object):
    """
    This class instantiates all the entities required to get a consistent
    representation of a humanoid robot:

    - robot model

    - angleEstimator used to link the two robot models

    - usual features and tasks for a robot:
     - center of mass
     - one task per operational point
    """

    OperationalPoints = ['left-wrist', 'right-wrist',
                         'left-ankle', 'right-ankle']
    """
    Operational points are specific interesting points of the robot
    used to control it.

    When an operational point is defined, signals corresponding to the
    point position and jacobian are created.

    For instance if creating an operational point for the left-wrist,
    the associated signals will be called "left-wrist" and
    "Jleft-wrist" for respectively the position and the jacobian.
    """

    name = None
    """Entity name (internal use)"""

    #FIXME: it should be some kind of global flag instead.
    simu = False
    """Are we in simulation or not?"""

    halfSitting = None
    """
    The half-sitting position is the robot initial pose.
    This attribute *must* be defined in subclasses.
    """

    dynamicRobot = None
    """
    The robot dynamic model.
    """
    dimension = None
    """The configuration size."""

    featureCom = None
    """
    This generic feature takes as input the robot center of mass
    and as desired value the featureComDes feature of this class.
    """
    featureComDes = None
    """
    The feature associated to the robot center of mass desired
    position.
    """
    comTask = None

    features = dict()
    """
    Features associated to each operational point. Keys are
    corresponding to operational points.
    """
    tasks = dict()
    """
    Features associated to each operational point. Keys are
    corresponding to operational points.
    """

    def loadModelFromKxml(self, name, filename):
        """
        Load a model from a kxml file and return the parsed model.
        This uses the Python parser class implement in
        dynamic_graph.sot.dynamics.parser.

        kxml is an extensible file format used by KineoWorks to store
        both the robot mesh and its kinematic chain.

        The parser also imports inertia matrices which is a
        non-standard property.
        """

        model = Parser(name, filename).parse()
        model.setProperty('ComputeVelocity', 'true')
        model.setProperty('ComputeCoM', 'true')
        model.setProperty('ComputeAccelerationCoM', 'false')
        model.setProperty('ComputeMomentum', 'false')
        model.setProperty('ComputeZMP', 'true')
        model.setProperty('ComputeBackwardDynamics', 'false')
        return model

    def loadModelFromJrlDynamics(self, name, modelDir, modelName,
                                 specificitiesPath, jointRankPath):
        """
        Load a model using the jrl-dynamics parser. This parser looks
        for VRML files in which kinematics and dynamics information
        have been added by extending the VRML format.

        It is mainly used by OpenHRP.

        Additional information are located in two different XML files.
        """
        #FIXME: add support for hrp2-10 here.
        model = DynamicHrp2(name)
        model.setFiles(modelDir, modelName,
                       specificitiesPath, jointRankPath)
        model.parse()
        return

    def initializeOpPoints(self, model, prefix):
        for op in self.OperationalPoints:
            model.createOpPoint(op, op)

    def initializeRobot(self):
        """
        If the robot model is correctly loaded, this method will then
        initialize the operational points, set the position to
        half-sitting with null velocity/acceleration.

        To finish, different tasks are initialized:
        - the center of mass task used to keep the robot stability
        - one task per operational point to ease robot control
        """
        if not self.dynamicRobot:
            raise "robots models have to be initialized first"

        if self.simu:
            self.robotSimu = RobotSimu(self.name + '.robotSimu')
            # Freeflyer reference frame should be the same as global
            # frame so that operational point positions correspond to
            # position in freeflyer frame.
            self.robotSimu.set(self.halfSitting)

        self.dynamicRobot.signal('position').value = self.halfSitting
        self.dynamicRobot.signal('velocity').value = self.dimension*(0.,)
        self.dynamicRobot.signal('acceleration').value = self.dimension*(0.,)

        self.initializeOpPoints(self.dynamicRobot,
                                self.name + '.dynamics')

        # --- center of mass ------------
        self.featureCom = FeatureGeneric(self.name + '.feature.com')
        plug(self.dynamicRobot.signal('com'), self.featureCom.signal('errorIN'))
        plug(self.dynamicRobot.signal('Jcom'),
             self.featureCom.signal('jacobianIN'))
        self.featureCom.signal('selec').value = '011'
        self.featureComDes = FeatureGeneric(self.name + '.feature.ref.com')
        self.featureComDes.signal('errorIN').value = (.0, .0, 0.)
        self.featureCom.signal('sdes').value = self.featureComDes
        self.comTask = Task(self.name + '.task.com')
        self.comTask.add(self.name + '.feature.com')
        self.comTask.signal('controlGain').value = 1.

        # --- operational points tasks -----
        self.features = dict()
        self.tasks = dict()
        for op in self.OperationalPoints:
            self.dynamicRobot.signal(op).recompute(0)
            self.features[op] = \
                FeaturePosition(self.name + '.feature.' + op,
                                self.dynamicRobot.signal(op),
                                self.dynamicRobot.signal('J' + op),
                                self.dynamicRobot.signal(op).value)
            self.tasks[op] = Task(self.name + '.task.' + op)
            self.tasks[op].add(self.name + '.feature.' + op)
            self.tasks[op].signal('controlGain').value = .2


    def __init__(self, name, simu):
        self.name = name
        if simu:
            self.simu = True
        else:
            self.simu = False


class HumanoidRobot(AbstractHumanoidRobot):

    halfSitting = [] #FIXME

    name = None
    simu = None
    filename = None

    def __init__(self, name, simu, filename):
        AbstractHumanoidRobot.__init__(self, name, simu)
        self.filename = filename
        self.dynamicRobot = \
            self.loadModelFromKxml (self.name + '.dynamics', self.filename)
        self.dimension = self.dynamicRobot.getDimension()
        self.halfSitting = self.dimension*(0.,)
        self.initializeRobot()
