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
from ovis_robotiq_gripper.msg import _OvisGripperPosition as positionMsg
from ovis_robotiq_gripper.msg import _OvisGripper_robot_output as outputMsg
from ovis_robotiq_gripper.msg import _OvisGripper_robot_input as inputMsg
import robotiq_modbus_rtu.comModbusRtu
import ovis_robotiq_gripper.baseRobotiq2FGripper
import rospy
import roslib
roslib.load_manifest('robotiq_2f_gripper_control')
roslib.load_manifest('robotiq_modbus_rtu')

# Global declararions
command = outputMsg.OvisGripper_robot_output()
current_position = inputMsg.OvisGripper_robot_input()


def command_callback(input):
    global command
    global current_position

    if input.position == 0:
        command.rACT = 0
        command.rGTO = 0

    elif input.position == 1:
        command.rACT = 1  # activates the gripper
        command.rGTO = 1  # Go to requested position
        command.rFR = 255  # maximum force
        command.rSP = 255  # Maximum speed

    elif input.position == 2 and current_position.gPR == 0:
        command.rPR = 255  # close

    elif input.position == 2 and current_position.gPR == 255:
        command.rPR = 0  # open

def boot_sequence(gripper, pubOutput):
    # Deactivate the gripper from its initial state
    reboot_part_1 = positionMsg.OvisGripperPosition
    reboot_part_1.position = 0
    command_callback(reboot_part_1)

    send_command(gripper, pubOutput)

    # Reactivate the gripper
    reboot_part_2 = positionMsg.OvisGripperPosition
    reboot_part_2.position = 1
    command_callback(reboot_part_2)

    send_command(gripper, pubOutput)

def send_command(gripper, pubOutput):
    global command

    # Publish the command
    pubOutput.publish(command)
    # Give the most recent command to the gripper class and send it
    gripper.refreshCommand(command)
    gripper.sendCommand()

def mainLoop(device):

    # Gripper is a 2F with a TCP connection
    gripper = ovis_robotiq_gripper.baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
    gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

    # We connect to the address received as an argument
    gripper.client.connectToDevice(device)

    rospy.init_node("ovis_robotiq_gripper")

    # The Gripper status is published on the topic named 'gripper_input'
    pubInput = rospy.Publisher(
        'gripper_input', inputMsg.OvisGripper_robot_input, queue_size=1)

    # The Gripper command is published on the topic named 'gripper_output'
    pubOutput = rospy.Publisher(
        'gripper_output', outputMsg.OvisGripper_robot_output, queue_size=1)

    boot_sequence(gripper, pubOutput)

    # We loop
    while not rospy.is_shutdown():
        global command
        global current_position

        # Get and publish the Gripper status
        status = gripper.getStatus()
        pubInput.publish(status)
        current_position = status

        # Get the most recent command
        rospy.Subscriber("/ovis/gripper/position_goal",
                         positionMsg.OvisGripperPosition, command_callback, queue_size=1)

        send_command(gripper, pubOutput)


if __name__ == '__main__':
    try:
        mainLoop(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
