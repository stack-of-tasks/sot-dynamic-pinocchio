# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

from __future__ import print_function

from functools import reduce

from dynamic_graph import plug
from dynamic_graph.sot.core.derivator import Derivator_of_Vector
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core.robot_simu import RobotSimu
from dynamic_graph.sot.dynamic_pinocchio import DynamicPinocchio
from dynamic_graph.tools import addTrace
from dynamic_graph.tracer_real_time import TracerRealTime

import pinocchio

class AbstractRobot(object):
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
                for c in e._children:
                    if hasattr(c, 'tag') and c.tag == 'mimic':
                        mimicJoints.append(name)
        self.removeJoints(mimicJoints)

    def removeJoints (self, joints):
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

    def loadModelFromString (self, urdfString, rootJointType=pinocchio.JointModelFreeFlyer,
            removeMimicJoints=True):
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

    def loadModelFromUrdf(self, urdfPath, urdfDir=None, rootJointType=pinocchio.JointModelFreeFlyer,
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
            n1 = 10 # len("package://")
            n2 = urdfPath.index(path.sep, n1)
            pkg = urdfPath[n1:n2]
            relpath = urdfPath[n2+1:]

            import rospkg
            rospack = rospkg.RosPack()
            abspkg = rospack.get_path(pkg)
            urdfFile = path.join(abspkg, relpath)
        else:
            urdfFile = urdfPath
        if urdfDir is None:
            import os
            urdfDir = os.environ["ROS_PACKAGE_PATH"].split(':')
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
            raise RuntimeError("robots models have to be initialized first")

        if not self.device:
            self.device = RobotSimu(self.name + '_device')
        """
        Robot timestep
        """
        self.timeStep = self.device.getTimeStep()

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
            self.dynamic.velocity.value = self.dimension * (0., )

        if self.enableAccelerationDerivator:
            self.accelerationDerivator = \
                Derivator_of_Vector('accelerationDerivator')
            self.accelerationDerivator.dt.value = self.timeStep
            plug(self.velocityDerivator.sout, self.accelerationDerivator.sin)
            plug(self.accelerationDerivator.sout, self.dynamic.acceleration)
        else:
            self.dynamic.acceleration.value = self.dimension * (0., )

        # self.initializeOpPoints()

        # TODO: hand parameters through srdf --- additional frames ---
        # self.frames = dict()
        # frameName = 'rightHand'
        # self.frames [frameName] = self.createFrame (
        #    "{0}_{1}".format (self.name, frameName),
        #    self.dynamic.getHandParameter (True), "right-wrist")
        # rightGripper is an alias for the rightHand:
        # self.frames ['rightGripper'] = self.frames [frameName]

        # frameName = 'leftHand'
        # self.frames [frameName] = self.createFrame (
        #    "{0}_{1}".format (self.name, frameName),
        #    self.dynamic.getHandParameter (False), "left-wrist")
        # leftGripper is an alias for the leftHand:
        # self.frames ["leftGripper"] = self.frames [frameName]

        # for (frameName, transformation, signalName) in self.AdditionalFrames:
        #    self.frames[frameName] = self.createFrame(
        #        "{0}_{1}".format(self.name, frameName),
        #        transformation,
        #        signalName)

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


class HumanoidRobot(AbstractHumanoidRobot):
    def __init__(self, name, pinocchio_model, pinocchio_data, initialConfig, OperationalPointsMap=None, tracer=None):
        AbstractHumanoidRobot.__init__(self, name, tracer)

        self.OperationalPoints.append('waist')
        self.OperationalPoints.append('chest')
        self.OperationalPointsMap = OperationalPointsMap

        self.dynamic = DynamicPinocchio(self.name + "_dynamic")
        self.dynamic.setModel(pinocchio_model)
        self.dynamic.setData(pinocchio_data)
        self.dimension = self.dynamic.getDimension()

        self.device = RobotSimu(self.name + "_device")

        self.device.resize(self.dynamic.getDimension())
        self.halfSitting = initialConfig
        self.device.set(self.halfSitting)
        plug(self.device.state, self.dynamic.position)

        if self.enableVelocityDerivator:
            self.velocityDerivator = Derivator_of_Vector('velocityDerivator')
            self.velocityDerivator.dt.value = self.timeStep
            plug(self.device.state, self.velocityDerivator.sin)
            plug(self.velocityDerivator.sout, self.dynamic.velocity)
        else:
            self.dynamic.velocity.value = self.dimension * (0., )

        if self.enableAccelerationDerivator:
            self.accelerationDerivator = \
                Derivator_of_Vector('accelerationDerivator')
            self.accelerationDerivator.dt.value = self.timeStep
            plug(self.velocityDerivator.sout, self.accelerationDerivator.sin)
            plug(self.accelerationDerivator.sout, self.dynamic.acceleration)
        else:
            self.dynamic.acceleration.value = self.dimension * (0., )
        if self.OperationalPointsMap is not None:
            self.initializeOpPoints()
