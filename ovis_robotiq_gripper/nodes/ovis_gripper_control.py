#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
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
#  * Neither the name of Robotiq, Inc. nor the names of its
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
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

import sys
import os
from sensor_msgs.msg import Joy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg
import robotiq_modbus_rtu.comModbusRtu
import robotiq_2f_gripper_control.baseRobotiq2FGripper
import rospy
import roslib
roslib.load_manifest('robotiq_2f_gripper_control')
roslib.load_manifest('robotiq_modbus_rtu')

# GLOBAL TEMP
command = outputMsg.Robotiq2FGripper_robot_output()


def joy_callback(data):
    global command

    button_B = data.buttons[1]
    button_X = data.buttons[2]
    button_Y = data.buttons[3]

    if button_Y:
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1  # activates the gripper
        command.rGTO = 1  # Go to requested position
        command.rFR = 255  # maximum force
        command.rSP = 255  # Maximum speed

    elif button_B:
        command.rPR = 255  # close

    elif button_X:
        command.rPR = 0  # open

def mainLoop(device):

    # Gripper is a 2F with a TCP connection
    gripper = robotiq_2f_gripper_control.baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
    gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

    # We connect to the address received as an argument
    gripper.client.connectToDevice(device)

    rospy.init_node("ovis_robotiq_gripper")

    # The Gripper status is published on the topic named 'Robotiq2FGripperRobotInput'
    pubInput = rospy.Publisher(
        'Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, queue_size=1)

    # The Gripper command is published on the topic named 'Robotiq2FGripperRobotOutput'
    pubOutput = rospy.Publisher(
        'Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)

    # The Gripper command is received from the topic named 'Robotiq2FGripperRobotOutput'
    #rospy.Subscriber('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, gripper.refreshCommand)

    # We loop
    while not rospy.is_shutdown():

        # Get and publish the Gripper status
        status = gripper.getStatus()
        pubInput.publish(status)

        # Wait a little
        # rospy.sleep(0.05)

        # Get the most recent command
        rospy.Subscriber("/joy", Joy, joy_callback, queue_size=1)

        # Send the most recent command
        pubOutput.publish(command)
        gripper.sendCommand()

        # Wait a little
        # rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        mainLoop(sys.argv[1])
    except rospy.ROSInterruptException:
        pass