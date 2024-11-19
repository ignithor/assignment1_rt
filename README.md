# assignment1_rt

Test on Ubuntu 20.04 with ROS Noetic

## Setup

Clone this repository in your Ros Workspace and compile with : 

```bash
catkin_make
```

Then source your Workspace with

```bash
source ./devel/setup.bash
```

## Run the differents Nodes

After calling 'roscore' you can run the first node to display the screen with the first turtle with

```bash
rosrun turtlesim turtlesim_node 
```

To run the first node of the assignment you have to open a new window and execute : 

```bash
rosrun assignment1_rt ui_control.py
```
To run the second node of the assignment you have to open a new window and execute : 

```bash
rosrun assignment1_rt distance_checking.py
```

If you want to control the first turtle manually with you keyboard, you can execute this command :

```bash
rosrun turtlesim turtle_teleop_key 
```

## Technical notes 

2 scripts python3

### ui_control.py

This node do :
- Spawn a new turtle in the environment: turtle2
- The user is able to select the robot they want to control (turtle1 or turtle2), and the velocity of the robot along x,y and around z.
- The command is send for 1 second, and then the robot stop, and the user can insert the command again. 

#### Node name : ui_control

- Publisher1 : 
    - Topic : /turtle1/cmd_vel
    - Type : geometry_msgs.msg.Twist
    - Queue_size : 10

- Publisher2 : 
    - Topic : /turtle2/cmd_vel
    - Type : geometry_msgs.msg.Twist
    - Queue_size : 10

### distance_checking.py

This node do : 
- Checks the euclidian distance between turtle1 and turtle2 and publish it on the topic /turtle_distance
- Stops the moving turtle if the two turtles are “too close” 
- Stops the moving turtle if the position is too close to the boundaries

#### Node name : distance_checking

- Subscriber1 :
    - Topic : /turtle1/pose
    - Type : turtlesim.msg.Pose

- Subscriber2 :
    - Topic : /turtle2/pose
    - Type : turtlesim.msg.Pose

- Publisher1 : 
    - Topic : /turtle_distance
    - Type : std_msgs.msg.Float32
    - Queue_size : 10

- Publisher2 : 
    - Topic : /turtle1/cmd_vel
    - Type : geometry_msgs.msg.Twist
    - Queue_size : 10

- Publisher3 : 
    - topic : /turtle2/cmd_vel
    - Type : geometry_msgs.msg.Twist
    - Queue_size : 10
