#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class DistanceNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('distance_checking', anonymous=True)
        
        # Parameters
        self.distance_threshold = 1.5  # Minimum distance between turtles
        self.boundary_threshold = 1.5  # Minimum distance to boundaries
        
        # Publishers
        self.distance_pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_pose_callback)
        rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_pose_callback)
        
        # Publisher for stopping turtle1
        self.turtle1_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Publisher for stopping turtle2
        self.turtle2_vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        
        # Pose data for turtles
        self.turtle1_pose = None
        self.turtle2_pose = None
        
        rospy.loginfo("Distance Node Initialized")

    def turtle1_pose_callback(self, data):
        self.turtle1_pose = data
        self.check_distance_and_boundaries()

    def turtle2_pose_callback(self, data):
        self.turtle2_pose = data
        self.check_distance_and_boundaries()

    def check_distance_and_boundaries(self):
        if self.turtle1_pose and self.turtle2_pose:
            # Calculate the Euclidean distance between the two turtles
            distance = ((self.turtle1_pose.x - self.turtle2_pose.x) ** 2 +
                        (self.turtle1_pose.y - self.turtle2_pose.y) ** 2) ** 0.5
            
            # Publish the distance
            self.distance_pub.publish(distance)
            
            # Check if the distance is below the threshold
            if distance < self.distance_threshold:
                rospy.logwarn("Turtles are too close! Stopping turtle.")
                self.stop_turtles()
                return
            
            # Check if turtle1 is near the boundaries
            if (self.turtle1_pose.x < self.boundary_threshold or
                self.turtle1_pose.x > 10.0 - self.boundary_threshold or
                self.turtle1_pose.y < self.boundary_threshold or
                self.turtle1_pose.y > 10.0 - self.boundary_threshold):
                rospy.logwarn("Turtle1 is near the boundary! Stopping turtles.")
                self.stop_turtles()
                return
            
            
            # Check if turtle2 is near the boundaries
            if (self.turtle2_pose.x < self.boundary_threshold or
                self.turtle2_pose.x > 10.0 - self.boundary_threshold or
                self.turtle2_pose.y < self.boundary_threshold or
                self.turtle2_pose.y > 10.0 - self.boundary_threshold):
                rospy.logwarn("Turtle2 is near the boundary! Stopping turtles.")
                self.stop_turtles()
                return

    def stop_turtles(self):
        # Stop turtles by publishing zero velocity
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.turtle1_vel_pub.publish(stop_msg)
        self.turtle2_vel_pub.publish(stop_msg)

if __name__ == '__main__':
    try:
        node = DistanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
