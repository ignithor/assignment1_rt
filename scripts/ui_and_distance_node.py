#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import time as t

class TurtleController:
    def __init__(self):
        # Initialize the node
        rospy.init_node("turtle_control_and_distance_check", anonymous=True)

        # Parameters
        self.distance_threshold = 1.5  # Minimum distance between turtles
        self.boundary_threshold = 1.5  # Minimum distance to boundaries

        # Spawn the second turtle
        rospy.wait_for_service("/spawn")
        self.spawn_client = rospy.ServiceProxy("/spawn", Spawn)
        self.spawn_client(2.0, 3.0, 0.0, "turtle2")

        # Publishers for turtle velocities
        self.turtle1_vel_pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)
        self.turtle2_vel_pub = rospy.Publisher("turtle2/cmd_vel", Twist, queue_size=10)

        # Subscriber for turtle poses
        rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_pose_callback)
        rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_pose_callback)

        # Distance publisher
        self.distance_pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)

        # Pose data for turtles
        self.turtle1_pose = None
        self.turtle2_pose = None

        # Control loop
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("Turtle Controller Initialized")

    def turtle1_pose_callback(self, data):
        self.turtle1_pose = data
        self.compute_turtle_distance()

    def turtle2_pose_callback(self, data):
        self.turtle2_pose = data
        self.compute_turtle_distance()
    
    def compute_turtle_distance(self):
        if self.turtle1_pose and self.turtle2_pose:
            self.distance_turtles = ((self.turtle1_pose.x - self.turtle2_pose.x) ** 2 + 
                        (self.turtle1_pose.y - self.turtle2_pose.y) ** 2) ** 0.5
            self.distance_pub.publish(Float32(self.distance_turtles))

    def will_increase_distance(self, vel, turtle_pose, target_pose=None):
        """
        Check if the given velocity will increase the distance to the target or boundary.

        Args:
            vel (Twist): Commanded velocity.
            turtle_pose (Pose): Current pose of the turtle.
            target_pose (Pose): Target pose of the other turtle (optional).

        Returns:
            bool: True if the velocity increases the distance, False otherwise.
        """
        # Predict future position of the turtle
        future_x = turtle_pose.x + vel.linear.x * 0.1
        future_y = turtle_pose.y + vel.linear.y * 0.1

        # Check if movement keeps the turtle within boundaries
        if (future_x < self.boundary_threshold or
            future_x > 11.0 - self.boundary_threshold or
            future_y < self.boundary_threshold or
            future_y > 11.0 - self.boundary_threshold):
            return False  # Future position violates boundary threshold

        # Check if movement increases distance to the other turtle
        if target_pose:
            current_distance = ((turtle_pose.x - target_pose.x) ** 2 +
                                (turtle_pose.y - target_pose.y) ** 2) ** 0.5
            future_distance = ((future_x - target_pose.x) ** 2 +
                               (future_y - target_pose.y) ** 2) ** 0.5
            if future_distance > current_distance:
                return True  # Movement increases the distance

            return False  # Movement reduces the distance

        # No target pose provided, assume safe movement
        return True

    def control_loop(self, event):
        try:
            # Get user input
            turtle_number = int(input("Enter the turtle number (1 or 2): "))
            if turtle_number not in [1, 2]:
                print("Invalid turtle number. Please enter 1 or 2.")
                return

            vel = Twist()
            vel.linear.x = float(input("Enter the linear velocity along x: "))
            vel.linear.y = float(input("Enter the linear velocity along y: "))
            vel.angular.z = float(input("Enter the angular velocity around z: "))

            # Determine which turtle to control
            if turtle_number == 1 and self.turtle1_pose and self.turtle2_pose:
                if self.distance_turtles <= self.distance_threshold:
                    # Block movement if the velocity will reduce the distance
                    if self.will_increase_distance(vel, self.turtle1_pose, self.turtle2_pose):
                        self.turtle1_vel_pub.publish(vel)
                    else:
                        rospy.logwarn("Turtle 1 is at the threshold, and velocity would reduce distance. Stopping Turtle 1.")
                        self.turtle1_vel_pub.publish(Twist())  # Stop turtle
                else:
                    # while distance is safe, allow movement
                    start_time = t.time()
                    while (self.distance_turtles > self.distance_threshold) and (t.time()-start_time < 1) and (11.0 - self.boundary_threshold > self.turtle1_pose.x > self.boundary_threshold) \
                            and (11.0 - self.boundary_threshold > self.turtle1_pose.y > self.boundary_threshold):
                        self.turtle1_vel_pub.publish(vel)
                        self.compute_turtle_distance()
                    self.turtle1_vel_pub.publish(Twist())
                    if self.distance_turtles <= self.distance_threshold or (11.0 - self.boundary_threshold < self.turtle1_pose.x) or ( self.turtle1_pose.x < self.boundary_threshold) \
                            or (11.0 - self.boundary_threshold < self.turtle1_pose.y) or ( self.turtle1_pose.y < self.boundary_threshold):
                        rospy.logwarn("Turtle 1 is at the threshold. Stopping Turtle 1.")


            elif turtle_number == 2 and self.turtle2_pose and self.turtle1_pose:

                if self.distance_turtles <= self.distance_threshold:
                    # Block movement if the velocity will reduce the distance
                    if self.will_increase_distance(vel, self.turtle2_pose, self.turtle1_pose):
                        self.turtle2_vel_pub.publish(vel)
                    else:
                        rospy.logwarn("Turtle 2 is at the threshold, and velocity would reduce distance. Stopping Turtle 2.")
                        self.turtle2_vel_pub.publish(Twist())  # Stop turtle
                else:
                    # If distance is safe, allow movement
                    start_time = t.time()
                    while (self.distance_turtles > self.distance_threshold) and (t.time()-start_time < 1) and (11.0 - self.boundary_threshold > self.turtle2_pose.x > self.boundary_threshold) \
                            and (11.0 - self.boundary_threshold > self.turtle2_pose.y > self.boundary_threshold):
                        self.turtle2_vel_pub.publish(vel)
                        self.compute_turtle_distance()
                    self.turtle2_vel_pub.publish(Twist())
                    if self.distance_turtles <= self.distance_threshold or (11.0 - self.boundary_threshold < self.turtle2_pose.x) or  (self.turtle2_pose.x < self.boundary_threshold) \
                            or (11.0 - self.boundary_threshold < self.turtle2_pose.y) or  (self.turtle2_pose.y< self.boundary_threshold):
                        rospy.logwarn("Turtle 2 is at the threshold. Stopping Turtle 2.")
            else:
                print("Invalid turtle number or missing pose data.")
        except Exception as e:
            rospy.logwarn(f"Error in control loop: {e}")

if __name__ == '__main__':
    try:
        TurtleController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
