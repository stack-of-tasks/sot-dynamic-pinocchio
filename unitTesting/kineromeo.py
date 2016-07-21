# ______________________________________________________________________________
# ******************************************************************************
#
# The simplest robot task: Just go and reach a point
#
# ______________________________________________________________________________
# ******************************************************************************
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
import dynamic_graph.script_shortcuts
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_tasks_kine import *
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from numpy import *

set_printoptions(suppress=True, precision=7)

#-----------------------------------------------------------------------------
#---- ROBOT SPECIFICATIONS----------------------------------------------------
#-----------------------------------------------------------------------------

#Define robotName, urdfpath and initialConfig

#robotName = 'romeo'
#urdfpath = ""

#initialConfig = (0, 0, .840252, 0, 0, 0,                                 # FF
#                 0,  0,  -0.3490658,  0.6981317,  -0.3490658,   0,       # LLEG
#                 0,  0,  -0.3490658,  0.6981317,  -0.3490658,   0,       # RLEG
#                 0,                                                      # TRUNK
#                 1.5,  0.6,  -0.5, -1.05, -0.4, -0.3, -0.2,              # LARM
#                 0, 0, 0, 0,                                             # HEAD
#                 1.5, -0.6,   0.5,  1.05, -0.4, -0.3, -0.2,              # RARM
#                 )

#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------
dyn = Dynamic("dyn")
dyn.setFile(urdfpath)
dyn.parse()
dyn.displayModel()

inertiaRotor = (0,)*6+(5e-4,)*31
gearRatio =  (0,)*6+(200,)*31
dyn.inertiaRotor.value = inertiaRotor
dyn.gearRatio.value    = gearRatio
dyn.velocity.value = robotDim*(0.,)
dyn.acceleration.value = robotDim*(0.,)

# ------------------------------------------------------------------------------
# --- ROBOT SIMULATION ---------------------------------------------------------
# ------------------------------------------------------------------------------



robot = RobotSimu(robotName)
robot.resize(robotDim)
dt=5e-3

robot.set( initialConfig )
addRobotViewer(robot, small=False)

plug(robot.state,dyn.position)

# ------------------------------------------------------------------------------
# ---- Kinematic Stack of Tasks (SoT)  -----------------------------------------
# ------------------------------------------------------------------------------
sot = SOT('sot')
sot.setSize(robotDim)
plug(sot.control,robot.control)


# ------------------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------
# ------------------------------------------------------------------------------
# ---- TASK -----------------------------

# ---- TASK GRIP
taskRH    = MetaTaskKine6d('rh',dyn,'rh','RWristPitch')
handMgrip = eye(4); handMgrip[0:3,3] = (0.1,0,0)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')
# --- STATIC COM (if not walking)
taskCom = MetaTaskKineCom(dyn)
dyn.com.recompute(0)
taskCom.featureDes.errorIN.value = dyn.com.value
taskCom.task.controlGain.value = 10


# --- CONTACTS
#define contactLF and contactRF
for name,joint in [ ['LF','LAnkleRoll'], ['RF','RAnkleRoll' ] ]:
    contact = MetaTaskKine6d('contact'+name,dyn,name,joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    contact.keep()
    locals()['contact'+name] = contact


target = (0.5,-0.2,1.3)
robot.viewer.updateElementConfig('zmp',target+(0,0,0))
gotoNd(taskRH,target,'111',(4.9,0.9,0.01,0.9))
sot.push(contactRF.task.name)
sot.push(contactLF.task.name)
sot.push(taskCom.task.name)
sot.push(taskRH.task.name)

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
@loopInThread
def inc():
    robot.increment(dt)
#    print "dyn_position_value_devel"
#    print asarray(dyn.position.value)
#    print "robot_control_value_devel"
#    print asarray(robot.control.value)
#    print "task_contactlf_jacobian_devel"
#    print transpose(asarray(contactLF.feature.signal("jacobian").value))
#    print "dyn_JLF_devel"
#    print transpose(asarray(dyn.signal("JLF").value))
runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

print "dyn_position_value_devel"
print asarray(dyn.position.value)
print "robot_control_value_devel"
print asarray(robot.control.value)
print "task_contactlf_jacobian_devel"
print transpose(asarray(contactLF.feature.signal("jacobian").value))
print "dyn_JLF_devel"
print transpose(asarray(dyn.signal("JLF").value))
go()
