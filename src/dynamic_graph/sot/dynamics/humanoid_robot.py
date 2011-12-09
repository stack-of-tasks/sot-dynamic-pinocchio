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

from __future__ import print_function

from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.tools import addTrace
from dynamic_graph.sot import SE3, R3, SO3
from dynamic_graph.sot.core import OpPointModifier
from dynamic_graph.sot.core.derivator import Derivator_of_Vector
from dynamic_graph.sot.core.feature_position import FeaturePosition
from dynamic_graph.sot.core import RobotSimu, FeaturePoint6dRelative, \
    FeatureGeneric, FeatureJointLimits, TaskPD, Constraint, GainAdaptive, SOT

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
                         'left-ankle', 'right-ankle',
                         'gaze']
    """
    Operational points are specific interesting points of the robot
    used to control it.

    When an operational point is defined, signals corresponding to the
    point position and jacobian are created.

    For instance if creating an operational point for the left-wrist,
    the associated signals will be called "left-wrist" and
    "Jleft-wrist" for respectively the position and the jacobian.
    """

    AdditionalFrames = []
    """
    Additional frames are frames which are defined w.r.t an operational point
    and provides an interesting transformation.

    It can be used, for instance, to store the sensor location.

    The contained elements must be triplets matching:
    - additional frame name,
    - transformation w.r.t to the operational point,
    - operational point file.
    """

    name = None
    """Entity name (internal use)"""

    halfSitting = None
    """
    The half-sitting position is the robot initial pose.
    This attribute *must* be defined in subclasses.
    """

    dynamic = None
    """
    The robot dynamic model.
    """
    device = None
    """
    The device that integrates the dynamic equation, namely
      - the real robot or
      - a simulator
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

    frames = dict()
    """
    Additional frames defined by using OpPointModifier.
    """

    #FIXME: the following options are /not/ independent.
    # zmp requires acceleration which requires velocity.
    """
    Enable velocity computation.
    """
    enableVelocityDerivator = False
    """
    Enable acceleration computation.
    """
    enableAccelerationDerivator = False
    """
    Enable ZMP computation
    """
    enableZmpComputation = False

    """
    Tracer used to log data.
    """
    tracer = None

    """
    How much data will be logged.
    """
    tracerSize = 2**20

    """
    Automatically recomputed signals through the use
    of device.after.
    This list is maintained in order to clean the
    signal list device.after before exiting.
    """
    autoRecomputedSignals = []

    """
    Which signals should be traced.
    """
    tracedSignals = {
        'dynamic': ["com", "zmp", "angularmomentum",
                  "position", "velocity", "acceleration"],
        'device': ['zmp', 'control', 'state']
        }

    """
    Robot timestep
    """
    timeStep = 0.005

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
        self.setProperties(model)
        return model

    def loadModelFromJrlDynamics(self, name, modelDir, modelName,
                                 specificitiesPath, jointRankPath,
                                 dynamicType):
        """
        Load a model using the jrl-dynamics parser. This parser looks
        for VRML files in which kinematics and dynamics information
        have been added by extending the VRML format.

        It is mainly used by OpenHRP.

        Additional information are located in two different XML files.
        """
        model = dynamicType(name)
        self.setProperties(model)
        model.setFiles(modelDir, modelName,
                       specificitiesPath, jointRankPath)
        model.parse()
        return model

    def setProperties(self, model):
        model.setProperty('TimeStep', str(self.timeStep))

        model.setProperty('ComputeAcceleration', 'false')
        model.setProperty('ComputeAccelerationCoM', 'false')
        model.setProperty('ComputeBackwardDynamics', 'false')
        model.setProperty('ComputeCoM', 'false')
        model.setProperty('ComputeMomentum', 'false')
        model.setProperty('ComputeSkewCom', 'false')
        model.setProperty('ComputeVelocity', 'false')
        model.setProperty('ComputeZMP', 'false')

        model.setProperty('ComputeAccelerationCoM', 'true')
        model.setProperty('ComputeCoM', 'true')
        model.setProperty('ComputeVelocity', 'true')
        model.setProperty('ComputeZMP', 'true')

        if self.enableZmpComputation:
            model.setProperty('ComputeBackwardDynamics', 'true')
            model.setProperty('ComputeAcceleration', 'true')
            model.setProperty('ComputeMomentum', 'true')


    def initializeOpPoints(self, model):
        for op in self.OperationalPoints:
            model.createOpPoint(op, op)

    def createCenterOfMassFeatureAndTask(self,
                                         featureName, featureDesName,
                                         taskName,
                                         selec = '011',
                                         gain = 1.):
        self.dynamic.com.recompute(0)
        self.dynamic.Jcom.recompute(0)

        featureCom = FeatureGeneric(featureName)
        plug(self.dynamic.com, featureCom.errorIN)
        plug(self.dynamic.Jcom, featureCom.jacobianIN)
        featureCom.selec.value = selec
        featureComDes = FeatureGeneric(featureDesName)
        featureComDes.errorIN.value = self.dynamic.com.value
        featureCom.setReference(featureComDes.name)
        comTask = TaskPD(taskName)
        comTask.add(featureName)
        comTask.controlGain.value = gain
        return (featureCom, featureComDes, comTask)

    def createOperationalPointFeatureAndTask(self,
                                             operationalPointName,
                                             featureName,
                                             taskName,
                                             gain = .2):
        jacobianName = 'J{0}'.format(operationalPointName)
        self.dynamic.signal(operationalPointName).recompute(0)
        self.dynamic.signal(jacobianName).recompute(0)
        feature = \
            FeaturePosition(featureName,
                            self.dynamic.signal(operationalPointName),
                            self.dynamic.signal(jacobianName),
                            self.dynamic.signal(operationalPointName).value)
        task = TaskPD(taskName)
        task.add(featureName)
        task.controlGain.value = gain
        return (feature, task)

    def createFrame(self, frameName, transformation, operationalPoint):
        frame = OpPointModifier(frameName)
        frame.setTransformation(transformation)
        plug(self.dynamic.signal(operationalPoint),
             frame.positionIN)
        plug(self.dynamic.signal("J{0}".format(operationalPoint)),
             frame.jacobianIN)
        frame.position.recompute(frame.position.time + 1)
        frame.jacobian.recompute(frame.jacobian.time + 1)
        return frame

    def initializeRobot(self):
        """
        If the robot model is correctly loaded, this method will then
        initialize the operational points, set the position to
        half-sitting with null velocity/acceleration.

        To finish, different tasks are initialized:
        - the center of mass task used to keep the robot stability
        - one task per operational point to ease robot control
        """
        if not self.dynamic:
            raise RunTimeError("robots models have to be initialized first")

        if not self.device:
            self.device = RobotSimu(self.name + '_device')

        # Freeflyer reference frame should be the same as global
        # frame so that operational point positions correspond to
        # position in freeflyer frame.
        self.device.set(self.halfSitting)
        self.dynamic.position.value = self.halfSitting

        if self.enableVelocityDerivator:
            self.velocityDerivator = Derivator_of_Vector('velocityDerivator')
            self.velocityDerivator.dt.value = self.timeStep
            plug(self.device.state, self.velocityDerivator.sin)
            plug(self.velocityDerivator.sout, self.dynamic.velocity)
        else:
            self.dynamic.velocity.value = self.dimension*(0.,)

        if self.enableAccelerationDerivator:
            self.accelerationDerivator = \
                Derivator_of_Vector('accelerationDerivator')
            self.accelerationDerivator.dt.value = self.timeStep
            plug(self.velocityDerivator.sout,
                 self.accelerationDerivator.sin)
            plug(self.accelerationDerivator.sout, self.dynamic.acceleration)
        else:
            self.dynamic.acceleration.value = self.dimension*(0.,)

        self.initializeOpPoints(self.dynamic)

        # --- center of mass ------------
        (self.featureCom, self.featureComDes, self.comTask) = \
            self.createCenterOfMassFeatureAndTask(
            '{0}_feature_com'.format(self.name),
            '{0}_feature_ref_com'.format(self.name),
            '{0}_task_com'.format(self.name))

        # --- operational points tasks -----
        self.features = dict()
        self.tasks = dict()
        for op in self.OperationalPoints:
            (self.features[op], self.tasks[op]) = \
                self.createOperationalPointFeatureAndTask(
                op, '{0}_feature_{1}'.format(self.name, op),
                '{0}_task_{1}'.format(self.name, op))
            # define a member for each operational point
            w = op.split('-')
            memberName = w[0]
            for i in w[1:]:
                memberName += i.capitalize()
            setattr(self, memberName, self.features[op])

        # --- additional frames ---
        self.frames = dict()
        for (frameName, transformation, signalName) in self.AdditionalFrames:
            self.frames[frameName] = self.createFrame(
                "{0}_{1}".format(self.name, frameName),
                transformation,
                signalName)

        # Initialize tracer.
        self.initializeTracer()

    def addTrace(self, entityName, signalName):
        self.autoRecomputedSignals.append(
            '{0}.{1}'.format(entityName, signalName))
        addTrace(self, self.tracer, entityName, signalName)

    def initializeTracer(self):
        if not self.tracer:
            self.tracer = TracerRealTime('trace')
            self.tracer.setBufferSize(self.tracerSize)
            self.tracer.open('/tmp/','dg_','.dat')
            # Recompute trace.triger at each iteration to enable tracing.
            self.device.after.addSignal('{0}.triger'.format(self.tracer.name))

        # Geometry / operational points
        for s in self.OperationalPoints + self.tracedSignals['dynamic']:
            self.addTrace(self.dynamic.name, s)

        # Geometry / frames
        for (frameName, _, _) in self.AdditionalFrames:
            for s in ['position', 'jacobian']:
                self.addTrace(self.frames[frameName].name, s)

        # Robot features
        for s in self.OperationalPoints:
            self.addTrace(self.features[s]._reference.name, 'position')
            self.addTrace(self.tasks[s].name, 'error')

        # Com
        self.addTrace(self.featureComDes.name, 'errorIN')
        self.addTrace(self.comTask.name, 'error')

        # Device
        for s in self.tracedSignals['device']:
            self.addTrace(self.device.name, s)
        if type(self.device) != RobotSimu:
            self.addTrace(self.device.name, 'robotState')

        # Misc
        if self.enableVelocityDerivator:
            self.addTrace(self.velocityDerivator.name, 'sout')
        if self.enableAccelerationDerivator:
            self.addTrace(self.accelerationDerivator.name, 'sout')


    def __init__(self, name, tracer = None):
        self.name = name

        # Initialize tracer if necessary.
        if tracer:
            self.tracer = tracer

    def __del__(self):
        if self.tracer:
            self.stopTracer()

    def startTracer(self):
        """
        Start the tracer if it does not already been stopped.
        """
        if self.tracer:
            self.tracer.start()

    def stopTracer(self):
        """
        Stop and destroy tracer.
        """
        self.tracer.dump()
        self.tracer.stop()
        self.tracer.close()
        self.tracer.clear()
        for s in self.autoRecomputedSignals:
            self.device.after.rmSignal(s)
        self.tracer = None

    def reset(self, posture = None):
        """
        Restart the control from another position.

        This method has not been extensively tested and
        should be used carefully.

        In particular, tasks should be removed from the
        solver before attempting a reset.
        """
        if not posture:
            posture = self.halfSitting
        self.device.set(posture)

        self.dynamic.com.recompute(self.device.state.time+1)
        self.dynamic.Jcom.recompute(self.device.state.time+1)
        self.featureComDes.errorIN.value = self.dynamic.com.value

        for op in self.OperationalPoints:
            self.dynamic.signal(op).recompute(self.device.state.time+1)
            self.dynamic.signal('J'+op).recompute(self.device.state.time+1)
            self.features[op].reference.value = self.dynamic.signal(op).value

class HumanoidRobot(AbstractHumanoidRobot):

    halfSitting = [] #FIXME

    name = None
    filename = None

    def __init__(self, name, filename, tracer = None):
        AbstractHumanoidRobot.__init__(self, name, tracer)
        self.filename = filename
        self.dynamic = \
            self.loadModelFromKxml (self.name + '_dynamics', self.filename)
        self.dimension = self.dynamic.getDimension()
        self.halfSitting = self.dimension*(0.,)
        self.initializeRobot()
