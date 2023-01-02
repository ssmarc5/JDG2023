#!/usr/bin/env python3
import serial
from serial.threaded import LineReader, ReaderThread
import time
import rospy
from std_msgs.msg import Int32MultiArray

# Port names need to be defined in function of arduino names
ARDUINO_PORT = '/dev/ttyACM0'

MENU = "\nPick a command(number) to send to Machine Des Jeux\n\n\
\t1. Send an absolute target angle position of 180 degrees\n\
\t2. Send a 90 angle increase command relative to current position \n\
\t3. Stop the current movement as soon as possible while respecting configured acceleration \n\
\t4. Write a new speed/acceleration configuration for stepper motors in EEPROM memory \n\
\t5. Read the current speed/acceleration configuration for stepper motors from EEPROM memory \n\
\t6. Write a new movement threshold configuration for both steppers and servos \n\
\t7. Read the current threshold configuration\n\
\t8. Write a new speed/acceleration configuration for servo motors in EEPROM memory \n\
\t9. Read the current speed/acceleration configuration for servo motors from EEPROM memory \n\
"

class MachineDesJeuxProtocol(LineReader):
    """
    Implement a protocol for reading & writing serial data from/to arduino. 
    This protocol must be used by the ReaderThread class in conjunction
    with a defined serial port
    """
    TARGET_CMD = 1
    MOVE_CMD = 2
    STOP_CMD = 3
    WRITE_STEP_AXIS_CFG_CMD = 4
    READ_STEP_AXIS_CFG_CMD = 5
    WRITE_MOVE_END_CFG_CMD = 6
    READ_MOVE_END_CFG_CMD = 7
    WRITE_SERVO_AXIS_CFG_CMD = 8
    READ_SERVO_AXIS_CFG_CMD = 9

    def connection_made(self, transport):
        super().connection_made(transport)
        self.name = self.transport.serial.name
        print("Serial port" + self.name + "successfully opened!")

    def handle_line(self, data):
        """ 
        Process data received from arduino
        """
        print("data received was:\n{data}")
        try:
            if data.isnumeric() and int(data) == 69:
                pass
        except Exception:
            pass

    def connection_lost(self, exc):
        """
        Function called when port is closed unexpectedly or context exits
        """
        if exc:
            print("Serial port" + self.name + "lost connection!")

    def move_abs_angle(self, axis1, axis2, axis3, axis4, axis5, axis6):
        """
        Sends serial command to reach absolute angles targets given 
        as parameters. Angles in degrees
        TODO add limits 
        """
        self.write_line(f'{self.TARGET_CMD},{axis1},{axis2},{axis3},{axis4},{axis5},{axis6}')
    
    def move_relative_angle(self, axis1, axis2, axis3, axis4, axis5, axis6):
        """
        Sends serial command to reach relative (to current position)
        angles targets given as parameters. Angles in degrees.
        TODO add limits 
        """
        self.write_line(f'{self.MOVE_CMD},{axis1},{axis2},{axis3},{axis4},{axis5},{axis6}')

    def stop_movement(self):
        """
        Sends serial command to stop current movement of all 6 axis. Servo
        axis will slow down and comeback to reach the position where the axis
        was when the stop signal was sent. Stepper will decelerate and stop.
        """
        self.write_line('{self.STOP_CMD}')
    
    def write_stepper_cfg(self, max_speed_axis1, accel_axis1,
                                max_speed_axis2, accel_axis2,
                                max_speed_axis3, accel_axis3):
        """
        Sends serial command configure the max speed and acceleration of
        stepper motors. Those config are also written to EEPROM, allowing
        the Arduino to use them when it reboots. If using a new arduino
        device, this function should be called before sending commands.
        Parameters are in deg/seconds
        """
        self.write_line('{self.WRITE_STEP_AXIS_CFG_CMD},{max_speed_axis1},{accel_axis1},\
                            {max_speed_axis2},{accel_axis2},{max_speed_axis3},{accel_axis3}')

    def read_stepper_cfg(self):
        """
        Send serial command to read the configured stepper speed/accel currently written to memory.
        """
        self.write_line('{self.READ_STEP_AXIS_CFG_CMD}')

    def write_move_threshold_config(self, stepper_threshold, servo_threshold):
        """
        Send serial command to configure the threshold after which the axis movement is considered
        completed. This will influence the moment when you receive the movement completion from
        the arduino. The higher the threshold, the sooner the arduino considers de movement done.
        The stepper threshold is in motor steps and the servo threshold is in microseconds. Both
        have a maximum value of 255.
        """
        self.write_line('{self.WRITE_MOVE_END_CFG_CMD},{stepper_threshold},{servo_threshold}')
    
    def read_move_threshold_config(self):
        """
        Send serial command to read the configured thresholds currently written to memory.
        """
        self.write_line('{self.READ_MOVE_END_CFG_CMD}')

    def write_servo_cfg(self, max_speed_axis4, accel_axis4,
                                max_speed_axis5, accel_axis5,
                                max_speed_axis6, accel_axis6):
        """
        Sends serial command configure the max speed and acceleration of
        servo motors. Those config are also written to EEPROM, allowing
        the Arduino to use them when it reboots. If using a new arduino
        device, this function should be called before sending commands.
        Parameters are in deg/seconds
        """
        self.write_line('{self.WRITE_SERVO_AXIS_CFG_CMD},{max_speed_axis4},{accel_axis4},\
                            {max_speed_axis5},{accel_axis5},{max_speed_axis6},{accel_axis6}')

    def read_servo_cfg(self):
        """
        Send serial command to read the configured servo speed/accel currently written to memory.
        """
        self.write_line('{self.READ_SERVO_AXIS_CFG_CMD}')

def callback(data, protocol):
    print(str(data))
    val, axis1, axis2, axis3, axis4, axis5, axis6 = data.data
    if val == 1:
        protocol.move_abs_angle(axis1, axis2, axis3, axis4, axis5, axis6)
    elif val == 2:
        protocol.move_relative_angle(axis1, axis2, axis3, axis4, axis5, axis6)
    elif val == 3:
        protocol.stop_movement()
    elif val == 4:
        protocol.write_stepper_cfg(15,15,60,60,60,60)
    elif val == 5:
        protocol.read_stepper_cfg()
    elif val == 6:
        protocol.write_move_threshold_config(5,5)
    elif val == 7:
        protocol.read_move_threshold_config()
    elif val == 8:
        protocol.write_servo_cfg(120,120,120,120,120,120)
    elif val == 9:
        protocol.read_servo_cfg()
    elif val == 10:
        protocol.move_abs_angle(0,0,0,-135,0,-150)

def machine_path_handler(protocol):     
    # Start computing of commands here and write on serial bus when new data is available
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("control/motorCmd", Int32MultiArray, callback, protocol)
    # starts the node
    # rospy.init_node('motor_node')
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('motor_node') 
    # Open the serial port
    while True:
        try:
            global port
            port = serial.Serial(ARDUINO_PORT, 115200)
            break
        except serial.serialutil.SerialException as e:
            print("pod_node: Serial port" + str(ARDUINO_PORT) + "couldn't be opened:\n" + str(e))
            time.sleep(3)

    with ReaderThread(port, MachineDesJeuxProtocol) as protocol:  
        machine_path_handler(protocol)