import rospy
import numpy
import time
import termios, sys, os
from dynamixel_workbench_msgs.srv import DynamixelCommand
TERMIOS = termios

## This function creates de service to control de dynamixel motors
## First we wait for the service to be available in case it its
## been used by another process.
## Next we initialize the service inside a try, in case it fails
## the whole process don’t fail.
## Then we send the command with the parameters we called the function, 
## and wait a time for the command to be executed, 
## then we return the result of the command. 
def jointCommand(command, id_num, addr_name, value, time):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

## This function reads the key is pressed and returns, to use this function 
## it is necessary the command window in which was called the script is in focus.
def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c

## This function prints the number and name of the joint is 
## currently on focus and being controlled.
def printJoint(numJoint):
    if numJoint == 6:
        print("6-waist")
    elif numJoint == 7:
        print("7-shoulder")
    elif numJoint == 8:
        print("8-elbow")
    elif numJoint == 9:
        print("9-wrist")
    elif numJoint == 10:
        print("10-griper")

## In the main function first we define in an array the number of joints and its 
## home an objective position, then we define the ID of the first motor, we 
## assume they are consecutive from there, then inside a try we configure the
## torque for each one of the motors, then inside an infinite loop we wait for
## a key to be pressed, if it is ‘w’ or ‘s’ they change the motor in focus, if it is
## ‘a’ or ‘d’ they change the position of the motor to its home or objective 
##position. 
if __name__ == '__main__':
    joints = [[1, 400, 512], [2, 400, 512], [3, 400, 512], [4, 400, 512],[5, 0, 512]]
    numJoint = 6
    printJoint(numJoint)
    try:
        jointCommand('', 6, 'Torque_Limit', 600, 0)
        jointCommand('', 7, 'Torque_Limit', 500, 0)
        jointCommand('', 8, 'Torque_Limit', 400, 0)
        jointCommand('', 9, 'Torque_Limit', 400, 0)
        jointCommand('', 10, 'Torque_Limit', 300, 0)
        while 1:
            key = getkey()
            if key == b'w':
                a = numJoint+1
                if (a > 10):
                    numJoint = 6
                else:
                    numJoint = numJoint+1
                printJoint(numJoint)
            if key == b's':
                a = numJoint-1
                if (a < 6):
                    numJoint = 10
                else:
                    numJoint = numJoint-1
                printJoint(numJoint)
            if key == b'a':
                jointCommand('', numJoint, 'Goal_Position', joints[numJoint-6][2], 1)
            if key == b'd':
                jointCommand('', numJoint, 'Goal_Position', joints[numJoint-6][1], 1)
        
    except rospy.ROSInterruptException:
        pass
