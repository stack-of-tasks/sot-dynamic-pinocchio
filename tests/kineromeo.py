# flake8: noqa
# ______________________________________________________________________________
# ******************************************************************************
#
# The simplest robot task: Just go and reach a point
#
# ______________________________________________________________________________
# ******************************************************************************
import pinocchio as pin
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.dynamics import *
# from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from numpy import *
# Taking input from pinocchio
from pinocchio.romeo_wrapper import RomeoWrapper

set_printoptions(suppress=True, precision=7)

# -----------------------------------------------------------------------------
# ---- ROBOT SPECIFICATIONS----------------------------------------------------
# -----------------------------------------------------------------------------

# Define robotName, urdfpath and initialConfig

# SET THE PATH TO THE URDF AND MESHES
urdfPath = "~/git/sot/pinocchio/models/romeo.urdf"
urdfDir = ["~/git/sot/pinocchio/models"]
pinocchioRobot = RomeoWrapper(urdfPath, urdfDir, pin.JointModelFreeFlyer())
robotName = 'romeo'
pinocchioRobot.initDisplay(loadModel=True)
pinocchioRobot.display(pinocchioRobot.q0)
initialConfig = (
    0,
    0,
    .840252,
    0,
    0,
    0,  # FF
    0,
    0,
    -0.3490658,
    0.6981317,
    -0.3490658,
    0,  # LLEG
    0,
    0,
    -0.3490658,
    0.6981317,
    -0.3490658,
    0,  # RLEG
    0,  # TRUNK
    1.5,
    0.6,
    -0.5,
    -1.05,
    -0.4,
    -0.3,
    -0.2,  # LARM
    0,
    0,
    0,
    0,  # HEAD
    1.5,
    -0.6,
    0.5,
    1.05,
    -0.4,
    -0.3,
    -0.2,  # RARM
)

# -----------------------------------------------------------------------------
# ---- PINOCCHIO MODEL AND DATA --------------------------------------------------------------------
# -----------------------------------------------------------------------------
# pinocchioModel = pin.buildModelFromUrdf(urdfpath, pin.JointModelFreeFlyer())
# pinocchioData = pinocchioModel.createData()

# -----------------------------------------------------------------------------
# ---- DYN --------------------------------------------------------------------
# -----------------------------------------------------------------------------
dyn = Dynamic("dyn")
dyn.setModel(pinocchioRobot.model)
dyn.setData(pinocchioRobot.data)

dyn.displayModel()

robotDim = dyn.getDimension()
inertiaRotor = (0, ) * 6 + (5e-4, ) * 31
gearRatio = (0, ) * 6 + (200, ) * 31
dyn.inertiaRotor.value = inertiaRotor
dyn.gearRatio.value = gearRatio
dyn.velocity.value = robotDim * (0., )
dyn.acceleration.value = robotDim * (0., )

# ------------------------------------------------------------------------------
# --- ROBOT SIMULATION ---------------------------------------------------------
# ------------------------------------------------------------------------------

robot = RobotSimu(robotName)
robot.resize(robotDim)
dt = 5e-3

# TODO: This configuration follows xyzQuat format for freeflyer. Do something about it
# initialConfig = zip(*(list(matrixToTuple(pinocchioRobot.q0))))[0]

robot.set(initialConfig)
plug(robot.state, dyn.position)

# ------------------------------------------------------------------------------
# ---- Kinematic Stack of Tasks (SoT)  -----------------------------------------
# ------------------------------------------------------------------------------
sot = SOT('sot')
sot.setSize(robotDim)
plug(sot.control, robot.control)

#--------------------------------DISPLAY-----------------------------------------

# ------------------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------
# ------------------------------------------------------------------------------
# ---- TASK -----------------------------

# ---- TASK GRIP
taskRH = MetaTaskKine6d('rh', dyn, 'rh', 'RWristPitch')
handMgrip = eye(4)
handMgrip[0:3, 3] = (0.1, 0, 0)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')
# --- STATIC COM (if not walking)
taskCom = MetaTaskKineCom(dyn)
dyn.com.recompute(0)
taskCom.featureDes.errorIN.value = dyn.com.value
taskCom.task.controlGain.value = 10

# --- CONTACTS
# define contactLF and contactRF
for name, joint in [['LF', 'LAnkleRoll'], ['RF', 'RAnkleRoll']]:
    contact = MetaTaskKine6d('contact' + name, dyn, name, joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    contact.keep()
    locals()['contact' + name] = contact

target = (0.5, -0.2, 1.3)

# addRobotViewer(robot, small=False)
# robot.viewer.updateElementConfig('zmp',target+(0,0,0))

gotoNd(taskRH, target, '111', (4.9, 0.9, 0.01, 0.9))
sot.push(contactRF.task.name)
sot.push(contactLF.task.name)
sot.push(taskCom.task.name)
sot.push(taskRH.task.name)

# -------------------------------------------------------------------------------
# ----- MAIN LOOP ---------------------------------------------------------------
# -------------------------------------------------------------------------------


def runner(n):
    for i in xrange(n):
        robot.increment(dt)
        pinocchioRobot.display(fromSotToPinocchio(robot.state.value))


runner(1000)
