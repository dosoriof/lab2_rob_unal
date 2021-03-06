# Laboratory 2 - Phantom X - ROS

Ricardo Galindo

Diego Osorio

## Forward Kinematics

### Model and measurements
Robot Phantom X <br>
[![Captura-de-pantalla-de-2022-05-13-10-22-16.png](https://i.postimg.cc/jd7TsXz0/Captura-de-pantalla-de-2022-05-13-10-22-16.png)](https://postimg.cc/bspWg1C3)
### DH Parameters
| Link | alpha |  a |  d | theta | offset |
|:----:|:-----:|:--:|:--:|:-----:|--------|
| 1    | 90°   | 0  | L1 | q1    | 0      |
| 2    | 0     | L2 | 0  | q2    | 90°    |
| 3    | 0     | L3 | 0  | q3    | 0      |
| 4    | 0     | L4 | 0  | q4    | 0      |
## ROS
We've created a python script which moves each of the joints between two positions, one home position and one objective position. These postions were defined as follows:
- Waist
  - Home: 0° (512 bits)
  - Objective: 32,6° (400 bits)
- Shoulder
  - Home: 0° (512 bits)
  - Objective: 32,6° (400 bits)
- Elbow
  - Home: 0° (512 bits)
  - Objective: 32,6° (400 bits)
- Wrist
  - Home: 0° (512 bits)
  - Objective: 32,6° (400 bits)


This script is located in the scripts folder of the px_robot package in this repository with the name ***boardOperation.py***. When running the script the Phantom X can be operated in the next way:
- You can tell the program which joint you want to move. In the console, the name of the operating joint is printed.
  - With the 'W' key you go to the next joint (if you are in Waist you go to Shoulder ; if you are in shoulder you go to elbow and so on).
  - With the 'S' key you go to the previous articulation (if you are in wrist you go to elbow and so on).
  - It can be done cyclically, that is, the next joint to Wrist is Waist; and that the one before Waist is Wrist.
- Pressing the 'D' key brings the operated joint to the target position.
- Pressing the 'A' key should bring the operated joint to the home position.

For the script to work first we have to run any of the next ROS launch files:
- px_controllers.launch - allows to move the robot
- px_rviz_dyna.launch - allows to move the robot but also creates a virtual visualization on rviz
Then we can run and use the script:
``` 
roslaunch px_robot px_rviz_dyna.launch
cd catkin_ws/src/px_robot/scripts
python boardOperation.py
```

### Code explanation
The code starts with the imports as follows:
``` python
import rospy
import numpy
import time
import termios, sys, os
from dynamixel_workbench_msgs.srv import DynamixelCommand
TERMIOS = termios
```
Then we create de function to send the commands to the motors through a ROS service. 
``` python
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
```
Then is the function to obtain the key pressed in the keyboard.
``` python
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
```
Then we create a function to print the joint in focus.
``` python
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
```
Then is the main function, this function contains an infinite while loop.
``` python
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
```
### Video
The video was uploaded to Youtube and can be found in the next link:
[Movement of Phantom X joints between two positions](https://youtu.be/AkXbdgU3pi4)

## Toolbox
We used the Peter Corke Toolbox in order to define and visualize a plot of de robot's model. First we define each of the Links with the function ***Link***, their inputs are the DH parameters that we defined in a previous section. With these Links created, then we create the robot by using the ***Serial Link*** function:

``` matlab
l = [14.5, 10.7, 10.7, 9]; % Links lenght
% Robot Definition RTB
L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');
```

Now, we have to orientate the TCP in the correct way:

``` matlab
% Tool orientation
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];
```

With the model defined, we can start to play with it. We can use the ***.teach*** function to plot the robot and move their Links with sliders.
``` matlab
PhantomX.teach()
```
Now we are going to plot some positions of the Robot with the ***.plot*** function, which arguments are the Links' values q:
- Home position q = [0 0 0 0] <br>
[![Captura-de-pantalla-de-2022-05-13-11-33-52.png](https://i.postimg.cc/ZnnXf1kg/Captura-de-pantalla-de-2022-05-13-11-33-52.png)](https://postimg.cc/dZMW3NVm)
- q = [30° 45° -30° 70°]  <br>
[![Captura-de-pantalla-de-2022-05-13-11-36-38.png](https://i.postimg.cc/9QL8kLbH/Captura-de-pantalla-de-2022-05-13-11-36-38.png)](https://postimg.cc/rDtN0C7f)
- q = [90° -20° -70° 15°]  <br>
[![Captura-de-pantalla-de-2022-05-13-11-37-55.png](https://i.postimg.cc/76k2nkzH/Captura-de-pantalla-de-2022-05-13-11-37-55.png)](https://postimg.cc/GBXtb6zV)

## Matlab connection

With Matlab we can create a script that moves each of the joints of the Phantom X. First we have to start the conection between Matlab an ROS with ***rosinit*** to be able to use the Services of the dynamixel motors of the robot. Then we create a client for the service ****dynamixel_workbench/dynamixel_command**** and the respective message for the service. Finally we can call the service with the parameters that we want:
``` matlab
%%
rosinit
%%
motorSvcClient = rossvcclient('dynamixel_workbench/dynamixel_command');%Client creation
motorCommandMsg = rosmessage(motorSvcClient);%Message creation
%%
motorCommandMsg.AddrName = "Goal_Position";
motorCommandMsg.Id = 1;
motorCommandMsg.Value = 400;
call(motorSvcClient,motorCommandMsg); %Service call
```


## Matlab + ROS + Toolbox

To make the relationship between ROS, Matlab and the toolbox, we are going to make use of information that we have already done previously. Basically it is to make the connection between two of the previous sections of the laboratory: the generation and graphing of the motor model in matlab with the toolbox and the publication of commands in the robot joints also from matlab.

### Robot plot

We define again each of the links with the DH parameters and we create the Robot with every links
``` matlab
l = [14.5, 10.7, 10.7, 9]; % Links lenght
% Robot Definition RTB
L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];
```
Then we decided to create an array ***q*** with all the positions of the links that we are going to visualize (5 positions proposed on the lab guide). Our ***pose*** variable determines which of these positions from the ***q*** array we are going to use. And finally we plot the Robot model with the position:
``` matlab
% Plotting
q = [ 0,  0,   0,   0,   0; 
    -20, 20, -20,  20,   0;
     30,-30,  30, -30,   0;
    -90, 15, -55,  17,   0;
    -90, 45, -55,  45, 100];
q_rad = deg2rad(q);
pose = 1;
PhantomX.plot(q_rad(pose,1:4),'notiles','noname');
hold on
ws = [-50 50];
trplot(eye(4),'rgb','arrow','length',15,'frame','0')
axis([repmat(ws,1,2) 0 60])
```

### ROS connection
When we have already created our plot we settle this selected position on the real Robot. For this pourpose we have to use the ROS services from the px_robot package as we did on the Matlab Connection section:
``` matlab
%% ROS Connection 
rosinit
%%
motorSvcClient = rossvcclient('dynamixel_workbench/dynamixel_command');%creacion del cliente
motorCommandMsg = rosmessage(motorSvcClient);%creacion del mensaje
```
Once we have created the service client, we just have to call the service for each one of the motors. As we want to change the position of the links we have to set the AddrName in "Goal_Position". We use a for structure in order to go through each Id motor and change the value. We can se that the Value must be a number in bits between 0 and 1023 and we have defined this values in previous steps in degrees units, that's why we need to make a mapping convertion between the value in degrees and the values in bits. For this we also need the information of the minimum and maximum value of each articulation. These values are -150° and 150° respectively. We do this convertion thanks to the function ***mapfun***. This function is not from matlab so we need the file ***mapfun.m*** (placed in the matlab folder) to be able to use it. 

``` matlab
%% ROS Connection 
rosinit
%%
motorSvcClient = rossvcclient('dynamixel_workbench/dynamixel_command');%creacion del cliente
motorCommandMsg = rosmessage(motorSvcClient);%creacion del mensaje
%%
motorCommandMsg.AddrName = "Goal_Position";
for i=1:length(q)
    motorCommandMsg.Id = i+5;
    motorCommandMsg.Value = round(mapfun(q(pose,i),-150,150,0,1023));
    call(motorSvcClient,motorCommandMsg);
end
```

We can see this matlab code in the ***Model_and_jointSvc.m*** from the folder with name matlab

## Conclusions
-	Matlab is a program that contains an immense variety of functionalities and tools to execute tasks of engineering, calculus and design, non the less its high demand in computing capabilities, and the price of the licenses make it difficult to implement in the industry.
-	ROS has two main languages C++ and python, C++ being preferred due to better performance. But Python has less complexity in code and no need for explicit building.
-	The use of ROS to control robots allow us to implement complex processes without deep knowledge of the technical operation of the equipment used, taking advantage of the programs the manufacturers or the community share openly, this allows to small teams to build and operate complex robotic systems in a short time.
-	The robotic arm Dynamixel Phantom allow students to physically experience with complex systems without the risks of industrial machinery, but with the complexity necessary to build confidence to explore and learn about the industrial robotic machinery
