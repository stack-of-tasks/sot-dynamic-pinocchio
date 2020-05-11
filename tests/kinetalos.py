# ______________________________________________________________________________
# ******************************************************************************
#
# The simplest robot task: Just go and reach a point
#
# ______________________________________________________________________________
# ******************************************************************************

import pinocchio as pin
from numpy import eye
# -----------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# ---- Kinematic Stack of Tasks (SoT)  -----------------------------------------
# ------------------------------------------------------------------------------
# ---- DYN --------------------------------------------------------------------
# -----------------------------------------------------------------------------
from pinocchio.robot_wrapper import RobotWrapper

from dynamic_graph import plug
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import (MetaTaskKine6d, MetaTaskKineCom, gotoNd)
from dynamic_graph.sot.core.sot import SOT
from dynamic_graph.sot.dynamic_pinocchio import fromSotToPinocchio
from dynamic_graph.sot.dynamic_pinocchio.humanoid_robot import HumanoidRobot

# -----------------------------------------------------------------------------
# SET THE PATH TO THE URDF AND MESHES
# Define robotName, urdfpath and initialConfig
dt = 5e-3
urdfPath = "/PATH/TO/talos-data/robots/talos_full_collision.urdf"
urdfDir = ["/PATH/TO/talos-data"]
robotName = 'TALOS'
OperationalPointsMap = {
    'left-wrist': 'arm_left_7_joint',
    'right-wrist': 'arm_right_7_joint',
    'left-ankle': 'leg_left_5_joint',
    'right-ankle': 'leg_right_5_joint',
    'gaze': 'head_2_joint',
    'waist': 'root_joint',
    'chest': 'torso_2_joint'
}
initialConfig = (
    0.,
    0.,
    0.648702,
    0.,
    0.,
    0.,  # Free flyer
    0.,
    0.,
    -0.453786,
    0.872665,
    -0.418879,
    0.,  # Left Leg
    0.,
    0.,
    -0.453786,
    0.872665,
    -0.418879,
    0.,  # Right Leg
    0.,
    0.,  # Chest
    0.261799,
    0.17453,
    0.,
    -0.523599,
    0.,
    0.,
    0.1,  # Left Arm
    0.,
    0.,
    0.,
    0.,
    0.,
    0.,
    0.,  # Left gripper
    -0.261799,
    -0.17453,
    0.,
    -0.523599,
    0.,
    0.,
    0.1,  # Right Arm
    0.,
    0.,
    0.,
    0.,
    0.,
    0.,
    0.,  # Right gripper
    0.,
    0.  # Head
)

# -----------------------------------------------------------------------------
# ---- ROBOT SPECIFICATIONS----------------------------------------------------
# -----------------------------------------------------------------------------

pinocchioRobot = RobotWrapper(urdfPath, urdfDir, pin.JointModelFreeFlyer())

pinocchioRobot.initDisplay(loadModel=True)
pinocchioRobot.display(fromSotToPinocchio(initialConfig))

robot = HumanoidRobot(robotName, pinocchioRobot.model, pinocchioRobot.data, initialConfig, OperationalPointsMap)

sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())
plug(sot.control, robot.device.control)

# ------------------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------

taskRH = MetaTaskKine6d('rh', robot.dynamic, 'rh', robot.OperationalPointsMap['right-wrist'])
handMgrip = eye(4)
handMgrip[0:3, 3] = (0.1, 0, 0)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')
# --- STATIC COM (if not walking)
taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
taskCom.featureDes.errorIN.value = robot.dynamic.com.value
taskCom.task.controlGain.value = 10

# --- CONTACTS
# define contactLF and contactRF
for name, joint in [['LF', robot.OperationalPointsMap['left-ankle']],
                    ['RF', robot.OperationalPointsMap['right-ankle']]]:
    contact = MetaTaskKine6d('contact' + name, robot.dynamic, name, joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    contact.keep()
    locals()['contact' + name] = contact

target = (0.5, -0.2, 1.0)

# addRobotViewer(robot, small=False)
# robot.viewer.updateElementConfig('zmp',target+(0,0,0))

gotoNd(taskRH, target, '111', (4.9, 0.9, 0.01, 0.9))
sot.push(contactRF.task.name)  # noqa TODO
sot.push(contactLF.task.name)  # noqa TODO
sot.push(taskCom.task.name)
sot.push(taskRH.task.name)

# -------------------------------------------------------------------------------
# ----- MAIN LOOP ---------------------------------------------------------------
# -------------------------------------------------------------------------------


def runner(n):
    for i in range(n):
        robot.device.increment(dt)
        pinocchioRobot.display(fromSotToPinocchio(robot.device.state.value))


# runner(3000)
