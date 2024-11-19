#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import time as t

def controller():
    rospy.init_node("UI")
    rospy.wait_for_service("/spawn")
    client = rospy.ServiceProxy("/spawn", Spawn)
    client(1.0, 5.0, 0.0, "turtle2")
    pub1 = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)
    pub2 = rospy.Publisher("turtle2/cmd_vel", Twist, queue_size=10)
    Pubs = [pub1, pub2]
    while not rospy.is_shutdown():
        turtle_number = int(input("Enter the turtle number (1 or 2): "))
        vel = Twist()
        vel.linear.x = float(input("Enter the linear velocity along x: "))
        vel.linear.y = float(input("Enter the linear velocity along y: "))
        vel.angular.z = float(input("Enter the angular velocity around z: "))
        start_time = t.time()
        while t.time()-start_time < 1:            
            Pubs[turtle_number-1].publish(vel)
        

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass