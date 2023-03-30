# BSD 2-Clause License

# Copyright (c) 2018-2023, CNRS
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy
import pinocchio
from dynamic_graph.sot.dynamic_pinocchio.dynamic import DynamicPinocchio
from dynamic_graph.sot.core.operator import Selec_of_vector
from dynamic_graph.sot.core import Integrator
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph import plug
from abc import ABC, abstractmethod

class AbstractRobot(ABC):
    """
    This class instantiates all the entities required to get a consistent
    representation of a robot, mainly:
      - device : to integrate velocities into angular control,
      - dynamic: to compute forward geometry and kinematics,
      - integrator: an entity that integrates the velocity of the robot
        and keeps an internal configuration the robot is controlled to,
      - selector: an entity that selects the actuated degrees of freedom
        of the robot to send as a position control to the device.
    """

    tracerSize = 2**20
    tracer = None
    rosParamName = "/robot_description"
    defaultFilename = None
    # Automatically recomputed signals through the use
    # of integrator.after.
    # This list is maintained in order to clean the
    # signal list device.after before exiting.
    autoRecomputedSignals = []

    def __init__(self, name, device=None, loadFromFile=False):
        self.name = name
        if loadFromFile:
            if self.defaultFilename is None:
                raise RuntimeError("Member defaultFilename should be defined.")
            print("Loading from file " + self.defaultFilename)
            self.loadModelFromUrdf(self.defaultFilename, rootJointType=None)
        else:
            print("Using ROS parameter \"/robot_description\"")
            import rospy
            if self.rosParamName not in rospy.get_param_names():
                raise RuntimeError('"' + self.rosParamName +
                                   '" is not a ROS parameter.')
            s = rospy.get_param(self.rosParamName)
            self.loadModelFromString(s, rootJointType=None)

        # Create rigid body dynamics model and data (pinocchio)
        self.dynamic = DynamicPinocchio(self.name + "_dynamic")
        self.dynamic.setModel(self.pinocchioModel)
        self.dynamic.displayModel()
        self.integrator = Integrator(self.name + "_integrator")
        self.integrator.setModel(self.pinocchioModel)
        self.dynamic.add_signals()
        self.dynamic.signal("velocity").value = \
            numpy.zeros(self.pinocchioModel.nv)
        self.dynamic.signal("acceleration").value = \
            numpy.zeros(self.pinocchioModel.nv)
        self.device = device
        if not device is None:
            self.selector = Selec_of_vector(self.name + "_selector")
            plug(self.integrator.signal("configuration"),
                 self.selector.signal("sin"))
            plug(self.selector.signal("sout"), self.device.signal("control"))
            self.timeStep = self.device.getTimeStep()
        plug(self.integrator.signal("configuration"),
             self.dynamic.signal("position"))

    def initializeEntities(self):
        if not self.device is None:
            # Set the device limits.
            def get(s):
                s.recompute(0)
                return s.value
            lb = get(self.dynamic.lowerJl)
            ub = get(self.dynamic.upperJl)
            actuatedJoints = self.getActuatedJoints()
            # Joint bounds
            lb_device = numpy.zeros(len(actuatedJoints))
            ub_device = numpy.zeros(len(actuatedJoints))
            for i,j in enumerate(actuatedJoints):
                lb_device[i] = lb[j]
                ub_device[i] = ub[j]
            self.device.setPositionBounds(lb_device , ub_device)
            # velocity bounds
            ub = get(self.dynamic.upperVl)
            ub_device = numpy.zeros(len(actuatedJoints))
            for i,j in enumerate(actuatedJoints):
                ub_device[i] = ub[j]
            self.device.setVelocityBounds(-ub_device, ub_device)
            # torque bounds
            ub = get(self.dynamic.upperTl)
            ub_device = numpy.zeros(len(actuatedJoints))
            for i,j in enumerate(actuatedJoints):
                ub_device[i] = ub[j]
            self.device.setTorqueBounds(-ub_device, ub_device)

    def _removeMimicJoints(self, urdfFile=None, urdfString=None):
        """Parse the URDF, extract the mimic joints and call removeJoints."""
        # get mimic joints
        import xml.etree.ElementTree as ET

        if urdfFile is not None:
            assert (
                urdfString is None
            ), "One and only one of input argument should be provided"
            root = ET.parse(urdfFile)
        else:
            assert (
                urdfString is not None
            ), "One and only one of input argument should be provided"
            root = ET.fromstring(urdfString)

        mimicJoints = list()
        for e in root.iter("joint"):
            if "name" in e.attrib:
                name = e.attrib["name"]
                for c in e:
                    if hasattr(c, "tag") and c.tag == "mimic":
                        mimicJoints.append(name)
        self.removeJoints(mimicJoints)

    def _storeRootJointType(self, rootJointType):
        if rootJointType == pinocchio.JointModelFreeFlyer:
            self.rootJointType = "freeflyer"
        elif rootJointType == pinocchio.JointModelPlanar:
            self.rootJointType = "planar"
        elif rootJointType is None:
            self.rootJointType = "fixed"
        else:
            raise TypeError(
                "rootJointType should be either JointModelFreeflyer, "
                "JointModelPlanar, or None."
            )

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
            self.pinocchioModel = pinocchio.buildReducedModel(
                self.pinocchioModel, jointIds, q
            )
            self.pinocchioData = pinocchio.Data(self.pinocchioModel)

    def loadModelFromString(
        self,
        urdfString,
        rootJointType=pinocchio.JointModelFreeFlyer,
        removeMimicJoints=True,
    ):
        """Load a URDF model contained in a string
        - param rootJointType: the root joint type. None for no root joint.
        - param removeMimicJoints: if True, all the mimic joints found in the model
          are removed.
        """
        if rootJointType is None:
            self.pinocchioModel = pinocchio.buildModelFromXML(urdfString)
        else:
            self.pinocchioModel = pinocchio.buildModelFromXML(
                urdfString, rootJointType()
            )
        self.pinocchioData = pinocchio.Data(self.pinocchioModel)
        if removeMimicJoints:
            self._removeMimicJoints(urdfString=urdfString)
        self._storeRootJointType(rootJointType)

    def loadModelFromUrdf(
        self,
        urdfPath,
        urdfDir=None,
        rootJointType=pinocchio.JointModelFreeFlyer,
        removeMimicJoints=True,
    ):
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
            relpath = urdfPath[n2 + 1 :]

            import rospkg

            rospack = rospkg.RosPack()
            abspkg = rospack.get_path(pkg)
            urdfFile = path.join(abspkg, relpath)
        else:
            urdfFile = urdfPath
        if urdfDir is None:
            import os

            urdfDir = os.environ.get("ROS_PACKAGE_PATH", "").split(":")
        if rootJointType is None:
            self.pinocchioModel = pinocchio.buildModelFromUrdf(urdfFile)
        else:
            self.pinocchioModel = pinocchio.buildModelFromUrdf(
                urdfFile, rootJointType()
            )
        self.pinocchioData = pinocchio.Data(self.pinocchioModel)
        if removeMimicJoints:
            self._removeMimicJoints(urdfFile=urdfFile)
        self._storeRootJointType(rootJointType)

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

    def initializeTracer(self):
        if not self.tracer:
            self.tracer = TracerRealTime("trace")
            self.tracer.setBufferSize(self.tracerSize)
            self.tracer.open("/tmp/", "dg_", ".dat")
            # Recompute trace.triger at each iteration to enable tracing.
            self.integrator.after.addSignal("{0}.triger".format(
                self.tracer.name))

    def addTrace(self, entityName, signalName):
        if self.tracer:
            self.autoRecomputedSignals.append("{0}.{1}".
                                              format(entityName, signalName))
            signal = f"{entityName}.{signalName}"
            filename = f"{entityName}-{signalName}".replace("/", "_")
            self.tracer.add(signal, filename)
            self.integrator.after.addSignal(signal)

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
                self.integrator.after.rmSignal(s)
            self.tracer = None
