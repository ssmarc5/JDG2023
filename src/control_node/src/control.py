#! /usr/bin/env python
import rospy
import math as m
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

DEFAULT_SPEED = 0
MIN_SPEED = -100
MAX_SPEED = 100

BUTTON_ID_A  = 0
BUTTON_ID_B  = 1
BUTTON_ID_X  = 2
BUTTON_ID_Y  = 3
BUTTON_ID_LB = 4
BUTTON_ID_RB = 5

AXIS_ID_L_STICK_X        = 0
AXIS_ID_L_STICK_Y        = 1
AXIS_ID_LT               = 2
AXIS_ID_R_STICK_X        = 3
AXIS_ID_R_STICK_Y        = 4
AXIS_ID_RT               = 5
AXIS_ID_CROSS_LEFT_RIGHT = 6
AXIS_ID_CROSS_UP_DOWN    = 7

BUTTON_RELEASED = 0
BUTTON_PRESSED  = 1
CROSS_LEFT_PRESSED  = 1
CROSS_RIGHT_PRESSED = -1
CROSS_UP_PRESSED   = 1
CROSS_DOWN_PRESSED = -1

# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim or any other robot

def map_range(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)

def get_max2(a, b):
    return a if a > b else b

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Motor commands
def callback(data):
    cmd = Int16MultiArray()
    x   =  data.axes[AXIS_ID_L_STICK_X]
    y   = -data.axes[AXIS_ID_L_STICK_Y]
    rot =  data.axes[AXIS_ID_R_STICK_X]

    angle = m.atan2(y, x)# % (2 * m.pi) # FIXME necessaire?
    speed = m.hypot(x, y)
    offset = m.pi / 4

    sin = m.sin(angle - offset)
    cos = m.cos(angle - offset)
    #max = get_max2(m.abs(sin), m.abs(cos)) # FIXME: pas necessaire

    # Ce facteur de correction limite la puissance maximale à 1
    if speed + m.abs(rot) > 1:
        correction = 1 / (speed + m.abs(rot))
    else:
        correction = 1

    #wheel1_raw = (speed * cos / max - rot) * correction # avant-gauche
    #wheel2_raw = (speed * sin / max + rot) * correction # avant-droite
    #wheel3_raw = (speed * cos / max + rot) * correction # arriere-droite
    #wheel4_raw = (speed * sin / max - rot) * correction # arriere-gauche

    wheel1_raw = (speed * cos - rot) * correction # avant-gauche
    wheel2_raw = (speed * sin + rot) * correction # avant-droite
    wheel3_raw = (speed * cos + rot) * correction # arriere-droite
    wheel4_raw = (speed * sin - rot) * correction # arriere-gauche

    wheel1 = map_range(wheel1_raw, -1, 1, MIN_SPEED, MAX_SPEED)
    wheel2 = map_range(wheel2_raw, -1, 1, MIN_SPEED, MAX_SPEED)
    wheel3 = map_range(wheel3_raw, -1, 1, MIN_SPEED, MAX_SPEED)
    wheel4 = map_range(wheel4_raw, -1, 1, MIN_SPEED, MAX_SPEED)
    #axe4 = map_range(data.axes[4],-1,1,30,-30)
    #axe6 = map_range(data.axes[5],-1,1,30,0)
    #seuil = 20

#    if data.buttons[5] == 1:
#        mode = int(CLOSE_CLAMP_CMD)
#        cmd.data = [mode, 0, 0, 0, 0, 0, 0]
#        pub.publish(cmd)
#    elif data.buttons[4] == 1:
#        mode = int(OPEN_CLAMP_CMD)
#        cmd.data = [mode, 0, 0, 0, 0, 0, 0]
#        pub.publish(cmd)
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
    if data.buttons[BUTTON_ID_A] == 1:
        speed = 0
        send_button_cmd = True
    elif data.buttons[BUTTON_ID_X] == BUTTON_PRESSED:
        speed = 25
        send_button_cmd = True
    elif data.buttons[BUTTON_ID_Y] == BUTTON_PRESSED:
        speed = 50
        send_button_cmd = True
    elif data.buttons[BUTTON_ID_B] == BUTTON_PRESSED:
        speed = 75
        send_button_cmd = True
    elif data.axes[AXIS_ID_CROSS_LEFT_RIGHT] == CROSS_LEFT_PRESSED:
        speed = -25
        send_button_cmd = True
    elif data.axes[AXIS_ID_CROSS_UP_DOWN] == CROSS_UP_PRESSED:
        speed = -50
        send_button_cmd = True
    elif data.axes[AXIS_ID_CROSS_LEFT_RIGHT] == CROSS_RIGHT_PRESSED:
        speed = -75
        send_button_cmd = True
    else:
        send_button_cmd = False

    if send_button_cmd:
        cmd.data = [speed, speed, speed, speed] #FIXME
    else:
        cmd.data = [wheel1, wheel2, wheel3, wheel4]

    #pub.publish(cmd) # FIXME uncomment

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
