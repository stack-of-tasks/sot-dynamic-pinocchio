# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

from __future__ import print_function

import sys

import pinocchio
from dynamic_graph import plug
from dynamic_graph.sot.core.derivator import Derivator_of_Vector
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core.robot_simu import RobotSimu
from dynamic_graph.tools import addTrace
from dynamic_graph.tracer_real_time import TracerRealTime

if sys.version_info.major == 2:
    from abc import ABCMeta, abstractmethod

    class ABC:
        __metaclass__ = ABCMeta
else:
    from abc import ABC, abstractmethod


class AbstractRobot(ABC):
    """
    This class instantiates all the entities required to get a consistent
    representation of a robot, mainly:
      - device : to integrate velocities into angular control,
      - dynamic: to compute forward geometry and kinematics,
      - zmpFromForces: to compute ZMP force foot force sensors,
      - stabilizer: to stabilize balanced motions

    Operational points are stored into 'OperationalPoints' list. Some of them
    are also accessible directly as attributes:
      - leftWrist,
      - rightWrist,
      - leftAnkle,
      - rightAnkle,
      - Gaze.

    Operational points are mapped to the actual joints in the robot model
    via 'OperationalPointsMap' dictionary.
    This attribute *must* be defined in the subclasses

    Other attributes require to be defined:
        - halfSitting: half-sitting position is the robot initial pose.
            This attribute *must* be defined in subclasses.

        - dynamic: The robot dynamic model.

        - device: The device that integrates the dynamic equation, namely
            the real robot or
            a simulator

        - dimension: The configuration size.
    """
    def _initialize(self):
        self.OperationalPoints = []
        """
        Operational points are specific interesting points of the robot
        used to control it.

        When an operational point is defined, signals corresponding to the
        point position and jacobian are created.

        For instance if creating an operational point for the left-wrist,
        the associated signals will be called "left-wrist" and
        "Jleft-wrist" for respectively the position and the jacobian.
        """

        self.AdditionalFrames = []
        """
        Additional frames are frames which are defined w.r.t an operational point
        and provides an interesting transformation.

        It can be used, for instance, to store the sensor location.

        The contained elements must be triplets matching:
        - additional frame name,
        - transformation w.r.t to the operational point,
        - operational point file.
        """

        self.frames = dict()
        """
        Additional frames defined by using OpPointModifier.
        """

        # FIXME: the following options are /not/ independent.
        # zmp requires acceleration which requires velocity.
        """
        Enable velocity computation.
        """
        self.enableVelocityDerivator = False
        """
        Enable acceleration computation.
        """
        self.enableAccelerationDerivator = False
        """
        Enable ZMP computation
        """
        self.enableZmpComputation = False
        """
        Tracer used to log data.
        """
        self.tracer = None
        """
        How much data will be logged.
        """
        self.tracerSize = 2**20
        """
        Automatically recomputed signals through the use
        of device.after.
        This list is maintained in order to clean the
        signal list device.after before exiting.
        """
        self.autoRecomputedSignals = []
        """
        Which signals should be traced.
        """
        self.tracedSignals = {
            'dynamic': ["com", "zmp", "angularmomentum", "position", "velocity", "acceleration"],
            'device': ['zmp', 'control', 'state']
        }

    def help(self):
        print(AbstractHumanoidRobot.__doc__)

    def _removeMimicJoints(self, urdfFile=None, urdfString=None):
        """ Parse the URDF, extract the mimic joints and call removeJoints. """
        # get mimic joints
        import xml.etree.ElementTree as ET
        if urdfFile is not None:
            assert urdfString is None, "One and only one of input argument should be provided"
            root = ET.parse(urdfFile)
        else:
            assert urdfString is not None, "One and only one of input argument should be provided"
            root = ET.fromstring(urdfString)

        mimicJoints = list()
        for e in root.iter('joint'):
            if 'name' in e.attrib:
                name = e.attrib['name']
                for c in e:
                    if hasattr(c, 'tag') and c.tag == 'mimic':
                        mimicJoints.append(name)
        self.removeJoints(mimicJoints)

    def removeJoints(self, joints):
        """
        - param joints: a list of joint names to be removed from the self.pinocchioModel
        """
        jointIds = list()
        for j in joints:
            if self.pinocchioModel.existJointName(j):
                jointIds.append(self.pinocchioModel.getJointId(j))
        if len(jointIds) > 0:
            q = pinocchio.neutral(self.pinocchioModel)
            self.pinocchioModel = pinocchio.buildReducedModel(self.pinocchioModel, jointIds, q)
            self.pinocchioData = pinocchio.Data(self.pinocchioModel)

    def loadModelFromString(self, urdfString, rootJointType=pinocchio.JointModelFreeFlyer, removeMimicJoints=True):
        """ Load a URDF model contained in a string
        - param rootJointType: the root joint type. None for no root joint.
        - param removeMimicJoints: if True, all the mimic joints found in the model are removed.
        """
        if rootJointType is None:
            self.pinocchioModel = pinocchio.buildModelFromXML(urdfString)
        else:
            self.pinocchioModel = pinocchio.buildModelFromXML(urdfString, rootJointType())
        self.pinocchioData = pinocchio.Data(self.pinocchioModel)
        if removeMimicJoints:
            self._removeMimicJoints(urdfString=urdfString)

    def loadModelFromUrdf(self,
                          urdfPath,
                          urdfDir=None,
                          rootJointType=pinocchio.JointModelFreeFlyer,
                          removeMimicJoints=True):
        """
        Load a model using the pinocchio urdf parser. This parser looks
        for urdf files in which kinematics and dynamics information
        have been added.
        - param urdfPath: a path to the URDF file.
        - param urdfDir: A list of directories. If None, will use ROS_PACKAGE_PATH.
        """
        if urdfPath.startswith("package://"):
            from os import path
            n1 = 10  # len("package://")
            n2 = urdfPath.index(path.sep, n1)
            pkg = urdfPath[n1:n2]
            relpath = urdfPath[n2 + 1:]

            import rospkg
            rospack = rospkg.RosPack()
            abspkg = rospack.get_path(pkg)
            urdfFile = path.join(abspkg, relpath)
        else:
            urdfFile = urdfPath
        if urdfDir is None:
            import os
            urdfDir = os.environ.get("ROS_PACKAGE_PATH", "").split(':')
        if rootJointType is None:
            self.pinocchioModel = pinocchio.buildModelFromUrdf(urdfFile)
        else:
            self.pinocchioModel = pinocchio.buildModelFromUrdf(urdfFile, rootJointType())
        self.pinocchioData = pinocchio.Data(self.pinocchioModel)
        if removeMimicJoints:
            self._removeMimicJoints(urdfFile=urdfFile)

    def initializeOpPoints(self):
        for op in self.OperationalPoints:
            self.dynamic.createOpPoint(op, self.OperationalPointsMap[op])

    def createFrame(self, frameName, transformation, operationalPoint):
        frame = OpPointModifier(frameName)
        frame.setTransformation(transformation)
        plug(self.dynamic.signal(operationalPoint), frame.positionIN)
        plug(self.dynamic.signal("J{0}".format(operationalPoint)), frame.jacobianIN)
        frame.position.recompute(frame.position.time + 1)
        frame.jacobian.recompute(frame.jacobian.time + 1)
        return frame

    def setJointValueInConfig(self, q, jointNames, jointValues):
        """
        q: configuration to update
        jointNames: list of existing joint names in self.pinocchioModel
        jointValues: corresponding joint values.
        """
        model = self.pinocchioModel
        for jn, jv in zip(jointNames, jointValues):
            assert model.existJointName(jn)
            joint = model.joints[model.getJointId(jn)]
            q[joint.idx_q] = jv

    @abstractmethod
    def defineHalfSitting(self, q):
        """
        Define half sitting configuration using the pinocchio Model (i.e.
        with quaternions and not with euler angles).

        method setJointValueInConfig may be usefull to implement this function.
        """
        pass

    def initializeRobot(self):
        """
        If the robot model is correctly loaded, this method will then
        initialize the operational points, set the position to
        half-sitting with null velocity/acceleration.

        To finish, different tasks are initialized:
        - the center of mass task used to keep the robot stability
        - one task per operational point to ease robot control
        """
        if not hasattr(self, 'dynamic'):
            raise RuntimeError("Dynamic robot model must be initialized first")

        if not hasattr(self, 'device') or self.device is None:
            # raise RuntimeError("A device is already defined.")
            self.device = RobotSimu(self.name + '_device')
        self.device.resize(self.dynamic.getDimension())
        """
        Robot timestep
        """
        self.timeStep = self.device.getTimeStep()

        # Compute half sitting configuration
        import numpy
        """
        Half sitting configuration.
        """
        self.halfSitting = pinocchio.neutral(self.pinocchioModel)
        self.defineHalfSitting(self.halfSitting)
        self.halfSitting = numpy.array(self.halfSitting[:3].tolist() + [0., 0., 0.]  # Replace quaternion by RPY.
                                       + self.halfSitting[7:].tolist())
        assert self.halfSitting.shape[0] == self.dynamic.getDimension()

        # Set the device limits.
        def get(s):
            s.recompute(0)
            return s.value

        def opposite(v):
            return [-x for x in v]

        self.dynamic.add_signals()
        self.device.setPositionBounds(get(self.dynamic.lowerJl), get(self.dynamic.upperJl))
        self.device.setVelocityBounds(-get(self.dynamic.upperVl), get(self.dynamic.upperVl))
        self.device.setTorqueBounds(-get(self.dynamic.upperTl), get(self.dynamic.upperTl))

        # Freeflyer reference frame should be the same as global
        # frame so that operational point positions correspond to
        # position in freeflyer frame.
        self.device.set(self.halfSitting)
        plug(self.device.state, self.dynamic.position)

        if self.enableVelocityDerivator:
            self.velocityDerivator = Derivator_of_Vector('velocityDerivator')
            self.velocityDerivator.dt.value = self.timeStep
            plug(self.device.state, self.velocityDerivator.sin)
            plug(self.velocityDerivator.sout, self.dynamic.velocity)
        else:
            self.dynamic.velocity.value = numpy.zeros([
                self.dimension,
            ])

        if self.enableAccelerationDerivator:
            self.accelerationDerivator = \
                Derivator_of_Vector('accelerationDerivator')
            self.accelerationDerivator.dt.value = self.timeStep
            plug(self.velocityDerivator.sout, self.accelerationDerivator.sin)
            plug(self.accelerationDerivator.sout, self.dynamic.acceleration)
        else:
            self.dynamic.acceleration.value = numpy.zeros([
                self.dimension,
            ])

    def addTrace(self, entityName, signalName):
        if self.tracer:
            self.autoRecomputedSignals.append('{0}.{1}'.format(entityName, signalName))
            addTrace(self, self.tracer, entityName, signalName)

    def initializeTracer(self):
        if not self.tracer:
            self.tracer = TracerRealTime('trace')
            self.tracer.setBufferSize(self.tracerSize)
            self.tracer.open('/tmp/', 'dg_', '.dat')
            # Recompute trace.triger at each iteration to enable tracing.
            self.device.after.addSignal('{0}.triger'.format(self.tracer.name))

    def traceDefaultSignals(self):
        # Geometry / operational points
        for s in self.OperationalPoints + self.tracedSignals['dynamic']:
            self.addTrace(self.dynamic.name, s)

        # Geometry / frames
        for (frameName, _, _) in self.AdditionalFrames:
            for s in ['position', 'jacobian']:
                self.addTrace(self.frames[frameName].name, s)

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

    def __init__(self, name, tracer=None):
        self._initialize()

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
        if self.tracer:
            self.tracer.dump()
            self.tracer.stop()
            self.tracer.close()
            self.tracer.clear()
            for s in self.autoRecomputedSignals:
                self.device.after.rmSignal(s)
            self.tracer = None

    def reset(self, posture=None):
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

        self.dynamic.com.recompute(self.device.state.time + 1)
        self.dynamic.Jcom.recompute(self.device.state.time + 1)

        for op in self.OperationalPoints:
            self.dynamic.signal(self.OperationalPointsMap[op]).recompute(self.device.state.time + 1)
            self.dynamic.signal('J' + self.OperationalPointsMap[op]).recompute(self.device.state.time + 1)


class AbstractHumanoidRobot(AbstractRobot):
    def __init__(self, name, tracer=None):
        AbstractRobot.__init__(self, name, tracer)

    def _initialize(self):
        AbstractRobot._initialize(self)
        self.OperationalPoints.extend(['left-wrist', 'right-wrist', 'left-ankle', 'right-ankle', 'gaze'])
