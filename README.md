# assignment1_rt

## Setup

Clone this repository in your ROS Workspace and compile with : 

```bash
catkin_make
```

Then source your Workspace with

```bash
source ./devel/setup.bash
```

## Run the differents Nodes

After calling 'roscore' in a terminal you can run the first node in another terminal to display the screen with the first turtle by running

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

Now you can give the velocity of one of the two turtles and see them moving.


**I also created a third node called 'turtle_control_and_distance_check' that does the same thing as the first two nodes simultaneously but in a better way to avoid erratic behavior (see Technical notes below). To run this node, do not execute the two previous commands; instead, do the following:** 

```bash
rosrun assignment1_rt ui_and_distance_node.py
```

## Technical notes 

Tested on WSL2 with Ubuntu 20.04 and ROS Noetic

This package contains 3 python3 scripts

### ui_control.py

This node:
- Spawns a new turtle in the environment: turtle2
- Makes the user able to select the turtle they want to control (turtle1 or turtle2), and the velocity of the robot along x,y and around z.
- Sends the command for 1 second, and then the robot stop, and the user can insert the command again. 

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

This node: 
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


### ui_and_distance_node.py


The main problem of the two previous nodes is that the first node can always send a non-zero velocity to the turtle and the second node, if the distance beetween the turtles or to a border is below the threshold sends a zero velocity message.

This architecture led to conflicting commands, causing erratic behavior as both nodes published to the same velocity topic simultaneously with different command.

To solve this, the new node:

- Combines the functionalities of the two nodes.
- Uses only one publisher per turtle for velocity commands.
- Validates velocity commands before publishing to ensure turtles behave predictably.


This node:
- Spawns a new turtle in the environment: turtle2
- Makes the user able to select the turtle they want to control (turtle1 or turtle2), and the velocity of the robot along x,y and around z.
- Sends the command for 1 second **if the euclidian distance between turtle1 and turtle2 won't be under the threshold or be near the border** , and then the robot stop, and the user can insert the command again. 
- At the threshold, a function called 'will_increase_distance' checks if the new velocity command will increase or decrease the distance to the threshold to avoid the blocking of the turtles. If the velocity command would decrease the distance further, it is replaced with a stop command (velocity = 0) to avoid collisions.
- Checks the euclidian distance between turtle1 and turtle2 and publish it on the topic /turtle_distance
- Stops the moving turtle if the two turtles are “too close” 
- Stops the moving turtle if the position is too close to the boundaries


#### Node name : turtle_control_and_distance_check

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