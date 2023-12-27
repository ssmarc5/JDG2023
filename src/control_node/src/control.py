#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

TARGET_CMD = 1
MOVE_CMD = 2
STOP_CMD = 3
WRITE_STEP_AXIS_CFG_CMD = 4
READ_STEP_AXIS_CFG_CMD = 5
WRITE_MOVE_END_CFG_CMD = 6
READ_MOVE_END_CFG_CMD = 7
WRITE_SERVO_AXIS_CFG_CMD = 8
READ_SERVO_AXIS_CFG_CMD = 9
OPEN_CLAMP_CMD = 10
CLOSE_CLAMP_CMD = 11
CUSTOM_CMD = 20

MIN_SPEED = -100
MAX_SPEED = 100


# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim or any other robot

def map_range(x, in_min, in_max, out_min, out_max):
  return int((x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Motor commands
def callback(data):
    cmd = Int16MultiArray()
    wheel1 = map_range(data.axes[1], -1, 1, MIN_SPEED, MAX_SPEED)
    wheel2 = map_range(data.axes[1], -1, 1, MIN_SPEED, MAX_SPEED)
    wheel3 = map_range(data.axes[1], -1, 1, MIN_SPEED, MAX_SPEED)
    wheel4 = map_range(data.axes[1], -1, 1, MIN_SPEED, MAX_SPEED)
    #axe4 = map_range(data.axes[4],-1,1,30,-30)
    #axe6 = map_range(data.axes[5],-1,1,30,0)
    #seuil = 20

#    if data.buttons[5] == 1:
#        mode = int(CLOSE_CLAMP_CMD)
#        cmd.data = [mode, 0, 0, 0, 0, 0, 0]
#        pub.publish(cmd)
#
#    elif data.buttons[4] == 1:
#        mode = int(OPEN_CLAMP_CMD)
#        cmd.data = [mode, 0, 0, 0, 0, 0, 0]
#        pub.publish(cmd)
#
#    elif data.buttons[1] == 1:
#        if data.axes[6] > 0.5:
#            acc = 30
#        elif data.axes[6] < -0.5:
#            acc = 120
#        else:
#            acc = 60
#
#        mode = int(WRITE_STEP_AXIS_CFG_CMD)
#        cmd.data = [mode, acc, acc, acc, acc, acc, acc]
#        pub.publish(cmd)
#
#    elif data.buttons[3] == 1:
#        if data.axes[7] < -0.5:
#            acc = 30
#        elif data.axes[7] > 0.5:
#            acc = 120
#        else:
#            acc = 60
#
#        mode = int(WRITE_SERVO_AXIS_CFG_CMD)
#        cmd.data = [mode, acc, acc, acc, acc, acc, acc]
#        pub.publish(cmd)

#     elif data.buttons[2] == 1:
#        mode = int(CUSTOM_CMD)
#        cmd.data = [mode, 0, 0, 0, 0, 0, 0]
#        pub.publish(cmd)

    #else:
        # Le bouton A inverse le sens des gachettes
    #    if data.buttons[0] == 1:
    #        axe3 = -axe3
    #        axe6 = -axe6
        
    #mode = int(MOVE_CMD)
    #cmd.data = [mode, axe1, axe2, axe3, axe4, axe5, axe6]
    cmd.data = [wheel1, wheel2, wheel3, wheel4]
    pub.publish(cmd)

# Intializes everything
def start():
    rospy.init_node('control_node')
    # publishing to motor and servo
    global pub
    pub = rospy.Publisher('control/motorCmd', Int16MultiArray, queue_size = 10)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    # rospy.init_node('motor_node')
    rospy.spin()

if __name__ == '__main__':
    start()
