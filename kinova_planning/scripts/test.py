#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import tf
import copy
import copy
import rospy
import moveit_commander
import std_msgs.msg
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
import actionlib
from control_msgs.msg import *
from math import *
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

#q0 = [-0.448631, 0.428483, -0.562865, 0.546182]
q0 = [-0.456173, 0.475889, -0.542395 ,0.520810]
Tq0 = tf.transformations.quaternion_matrix(q0)
#p0= [0.1103, -0.0334, 0.0108]
p0 = [0.0981198, -0.0205410, 0.0710983]
Tq0[0][3] = p0[0]
Tq0[1][3] = p0[1]
Tq0[2][3] = p0[2]
q00 = tf.transformations.quaternion_multiply([0,sin(-pi/4),0,cos(-pi/4)],[0,0,sin(pi/2),cos(pi/2)])
Tq00 = tf.transformations.quaternion_matrix(q00)

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True



class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    self.cene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    self.group_name = "arm"
    self.group = moveit_commander.MoveGroupCommander(self.group_name)
    # self.desired_pose = 

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # self.pose_subscriber = rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, self.PoseCb)
    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    self.planning_frame = self.group.get_planning_frame()
    print "============ Reference frame: %s" % self.planning_frame

    # We can also print the name of the end-effector link for this group:
    self.eef_link = self.group.get_end_effector_link()
    print "============ End effector: %s" % self.eef_link

    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()
    print "============ Robot Groups:", self.robot.get_group_names()

    self.camera_pose = geometry_msgs.msg.Pose()
    self.sub = rospy.Subscriber('/visp_auto_tracker/object_position', geometry_msgs.msg.PoseStamped, self.callback)
    self.qr_pose = geometry_msgs.msg.Pose()


    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    # print "============ Printing robot state"
    # print self.robot.get_current_state()
    # print ""
    ## END_SUB_TUTORIAL

    # Misc variables
  # def PoseCb(self, msg) :


  def go_to_joint_state(self,state):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    group.go(state, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()
    

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(state, current_joints, 0.01)


  def go_to_pose_goal(self,pose):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    orientation_e = tf.transformations.euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w],'rzyx')
    orientation_e = [atan2(pose.position.y,pose.position.x),orientation_e[1],orientation_e[2]]
    orientation = tf.transformations.quaternion_from_euler(orientation_e[0],orientation_e[1],orientation_e[2],'rzyx')
    # print orientation_q
    pose.orientation.w = orientation[3]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]

    # l = sqrt(pose.position.x**2+pose.position.y**2)
    # if l > 0.3:
    #     dh = (l-0.3)*2/10
    #     pose.position.z = pose.position.z + dh

    
    group.set_pose_target(pose)



    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # # It is always good to clear your targets after planning with poses.
    # # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    # current_pose = self.group.get_current_pose().pose
    # return all_close(pose_goal, current_pose, 0.01)


  def shift_pose_goal(self, axis, value):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    group.shift_pose_target(axis, value)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # # It is always good to clear your targets after planning with poses.
    # # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    # current_pose = self.group.get_current_pose().pose
    # return all_close(pose_goal, current_pose, 0.01)

  def set_rpy_goal(self, xyz):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    group.set_rpy_target(xyz)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # # It is always good to clear your targets after planning with poses.
    # # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

  def set_position_goal(self, pos):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    x=pos.position.x
    y=pos.position.y
    z=pos.position.z
    print x,y,z
    group.set_position_target([x,y,z])

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # # It is always good to clear your targets after planning with poses.
    # # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    

 
  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    group.execute(plan, wait=True)

  def callback(self, msg):
    pose = msg.pose
    self.qr_pose = pose    
  

def gripper_client(client,distance):

    client.wait_for_server()
    rospy.loginfo("Server connected")

    # Creates a goal to send to the action server.
    goal = GripperCommandGoal()
    goal.command.position = distance

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    #client.wait_for_result()
    rospy.loginfo("Action finished")

def pose_matrix(pose):
    
    T = tf.transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    T[0][3] = pose.position.x
    T[1][3] = pose.position.y
    T[2][3] = pose.position.z
    return T

def pose_from_matrix(T):
    pose = geometry_msgs.msg.Pose()
    orientation = tf.transformations.quaternion_from_matrix(T)
    pose.position.x = T[0][3]
    pose.position.y = T[1][3]
    pose.position.z = T[2][3]
    pose.orientation.w = orientation[3]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    return pose



def main():
  try:
    # print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    # raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()
    
    client = actionlib.SimpleActionClient('gripper_action', GripperCommandAction)

    tutorial.group.set_goal_position_tolerance(0.003)
    tutorial.group.set_goal_orientation_tolerance(0.007)
    tutorial.group.set_goal_joint_tolerance(0.0001)
    wpose1 = tutorial.group.get_current_pose().pose
    order = 0

    
    joint = [0.0,-pi/2,0.0,pi/2,0.0]
    tutorial.go_to_joint_state(joint)
    while not rospy.is_shutdown():
        print "input"
        p = raw_input()
        if p == "-1":
            break

        qr_pose = tutorial.qr_pose
        Tc = pose_matrix(qr_pose)
        T = np.dot(np.dot(Tq0,Tc),Tq00)
        desired_pose = pose_from_matrix(T)

        desired_pose.position.x = desired_pose.position.x+ 0.05*cos(atan2(desired_pose.position.y,desired_pose.position.x))
        desired_pose.position.y = desired_pose.position.y+ 0.05*sin(atan2(desired_pose.position.y,desired_pose.position.x))
       
        desired_pose.position.z = desired_pose.position.z + 0.09
        print "1"
        print desired_pose
        first_pose = geometry_msgs.msg.Pose()
        first_pose = copy.deepcopy(desired_pose)
        first_pose.position.x = first_pose.position.x - 0.1*cos(atan2(first_pose.position.y,first_pose.position.x))
        first_pose.position.y = first_pose.position.y - 0.1*sin(atan2(first_pose.position.y,first_pose.position.x))
        print "2"
        print desired_pose
        tutorial.go_to_pose_goal(first_pose)
        rospy.sleep(0.5)
        tutorial.go_to_pose_goal(desired_pose)
        rospy.sleep(0.5)
        if order == 0:
          gripper_client(client,0.01)
        else:
          gripper_client(client,0.03)

        rospy.sleep(1.0)
        tutorial.go_to_pose_goal(first_pose)
        rospy.sleep(0.5)
        joint = [0.0,-pi/2,0.0,pi/2,0.0]
        tutorial.go_to_joint_state(joint)
        if order == 0:
          order = 1
        else:
          order = 0


  
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

