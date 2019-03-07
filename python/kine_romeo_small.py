# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_tasks_kine import *
from numpy import *

# Create the robot romeo.
from dynamic_graph.sot.romeo.robot import *
robot = Robot('romeo_small', device=RobotSimu('romeo_small'))

# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a simple kinematic solver.
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize ( robot )

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
# define the macro allowing to run the simulation.
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=5e-3
@loopInThread
def inc():
    robot.device.increment(dt)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)

# ---- TASKS -------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------

# ---- TASK GRIP ---
# Defines a task for the right hand.
taskRH=MetaTaskKine6d('rh',robot.dynamic,'right-wrist','right-wrist')
#  Move the operational point.
handMgrip=eye(4); handMgrip[0:3,3] = (0.1,0,0)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')
# robot.tasks['right-wrist'].add(taskRH.feature.name)

# --- STATIC COM (if not walking)
# create the com task and feature tasks.
solver = initialize(robot)
# remove the tasks from the sot (they've been automatically added by previous method).
solver.sot.clear()

# --- CONTACTS
# define contactLF and contactRF
for name,joint in [ ['LF','left-ankle'], ['RF','right-ankle' ] ]:
    contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    contact.keep()
    locals()['contact'+name] = contact

# --- RUN ----------------------------------------------------------------------

# Move the desired position of the right hand
# 1st param: task concerned
# 2st param: objective
# 3rd param: selected parameters
# 4th param: gain
target=(0.5,-0.2,0.8)
gotoNd(taskRH,target,'111',(4.9,0.9,0.01,0.9))

# Fill the stack with some tasks.
solver.push(contactRF.task)
solver.push(contactLF.task)
solver.push (robot.tasks ['com'])

solver.push(taskRH.task)

# type inc to run one iteration, or go to run indefinitely.
# go()

