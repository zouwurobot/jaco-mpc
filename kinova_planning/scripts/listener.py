#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import geometry_msgs.msg
from std_msgs.msg import String
import tf
from math import *
import numpy as np

q0  = tf.transformations.quaternion_multiply([0,sin(pi/4),0,cos(-pi/4)],[0,0,sin(-pi/4),cos(-pi/4)])
Tq0 = tf.transformations.quaternion_matrix(q0)
q00 = tf.transformations.quaternion_multiply([0,sin(-pi/4),0,cos(-pi/4)],[0,0,sin(pi/2),cos(pi/2)])
Tq00 = tf.transformations.quaternion_matrix(q00)

def callback(msg):
    pose = msg.pose
    position_x = pose.position.x
    position_y = pose.position.y
    pose.position.x = pose.position.z
    pose.position.y = -position_x
    pose.position.z = -position_y
    q = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    Tq = tf.transformations.quaternion_matrix(q)
    T = np.dot(np.dot(Tq0,Tq),Tq00)
    orientation = tf.transformations.euler_from_matrix(T, axes='rzyx')
    # orientation_e = tf.transformations.euler_from_quaternion(w,'rzyx')
    # orientation_q1 = tf.transformations.euler_from_quaternion(q1,'rzyx')
    

    print Tq0

    # print orientation_q0
    # print Tq0


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/visp_auto_tracker/object_position', geometry_msgs.msg.PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
