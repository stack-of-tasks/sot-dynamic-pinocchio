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

from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY
from dynamic_graph.sot.core.meta_tasks_kine import *
from numpy import *

# Create the robot romeo.
from dynamic_graph.sot.romeo.romeo import *
robot = Robot('romeo', device=RobotSimu('romeo'))

plug(robot.device.state, robot.dynamic.position)

# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a simple kinematic solver.
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize

# Alternate visualization tool
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer
addRobotViewer(robot.device,small=True,small_extra=24,verbose=False)

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

