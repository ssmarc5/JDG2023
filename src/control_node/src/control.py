#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

TARGET_CMD = 1
MOVE_CMD = 2
STOP_CMD = 3
WRITE_STEP_AXIS_CFG_CMD = 4
READ_STEP_AXIS_CFG_CMD = 5
WRITE_MOVE_END_CFG_CMD = 6
READ_MOVE_END_CFG_CMD = 7
WRITE_SERVO_AXIS_CFG_CMD = 8
READ_SERVO_AXIS_CFG_CMD = 9


# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim or any other robot

def map_range(x, in_min, in_max, out_min, out_max):
  return int((x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Motor commands
def callback(data):
    cmd = Int32MultiArray()
    print(str(data.axes[0]))
    axe1 = map_range(data.axes[0],-1,1,0,270)
    axe2 = map_range(data.axes[1],-1,1,0,270)
    axe3 = map_range(data.axes[2],-1,1,0,270)
    axe4 = map_range(data.axes[3],-1,1,0,270)
    axe5 = map_range(data.axes[4],-1,1,0,270)
    axe6 = map_range(data.axes[5],-1,1,0,270)
    mode = int(TARGET_CMD)
    cmd.data = [mode, axe1, axe2, axe3, axe4, axe5, axe6]
    
    pub.publish(cmd)

# Intializes everything
def start():
    rospy.init_node('control_node') 
    # publishing to motor and servo
    global pub
    pub = rospy.Publisher('control/motorCmd', Int32MultiArray, queue_size = 10 )
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    # rospy.init_node('motor_node')
    rospy.spin()

if __name__ == '__main__':
    start()