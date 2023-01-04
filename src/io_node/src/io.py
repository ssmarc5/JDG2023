#!/usr/bin/env python3

# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32MultiArray

# Pin Definitions
output_pin_ledGreen = 18  # BCM pin 18, BOARD pin 12
output_pin_ledYellow = 23  # BCM pin 23, BOARD pin 16
output_pin_ledBlue = 24  # BCM pin 24, BOARD pin 18
input_pin_switchPince = 25  # BCM pin 25, BOARD pin 22
input_pin_switchSize = 8  # BCM pin 8, BOARD pin 24
input_pin_switchShaft = 7  # BCM pin 7, BOARD pin 26


def callback(data):
    led1, led2, led3 = data.data
    GPIO.output(output_pin_ledGreen, led1)
    GPIO.output(output_pin_ledYellow, led2)
    GPIO.output(output_pin_ledBlue, led3)

def setup():
    GPIO.setwarnings(False)
    # Pin Setup:
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    GPIO.setup(input_pin_switchPince, GPIO.IN)
    GPIO.setup(input_pin_switchSize, GPIO.IN)
    GPIO.setup(input_pin_switchShaft, GPIO.IN)
    GPIO.setup(output_pin_ledGreen, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(output_pin_ledYellow, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(output_pin_ledBlue, GPIO.OUT, initial=GPIO.HIGH)


def read_input(pub):
    cmd = Int32MultiArray()
    int1 = GPIO.input(input_pin_switchPince)
    int2 = GPIO.input(input_pin_switchSize)
    int3 = GPIO.input(input_pin_switchShaft)
    cmd.data = [int1, int2, int3]
    pub.publish(cmd)


def main():
    setup()
    rospy.init_node('io_node')
    pub = rospy.Publisher('io/ioState', Int32MultiArray, queue_size=10)
    # subscribed to io Cmd inputs on topic "control/ioCmd"
    rospy.Subscriber("control/ioCmd", Int32MultiArray, callback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        read_input(pub)
        rate.sleep()


if __name__ == '__main__':
    main()
