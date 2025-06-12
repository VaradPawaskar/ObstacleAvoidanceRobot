#!/usr/bin/env python

"""
Author: Varad Pawaskar
Description: Custom ROS node for autonomous obstacle avoidance using an ultrasonic sensor.
This script reads distance data and publishes motor commands to navigate without collisions.
"""

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('smart_navigator', anonymous=True)

        # Publisher to control robot velocity
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to the ultrasonic sensor distance
        self.distance_subscriber = rospy.Subscriber('/ultrasonic_distance', Float32, self.callback_distance)

        # Initialize distance and command message
        self.current_distance = 100.0  # Default safe distance
        self.cmd = Twist()

        rospy.loginfo("Smart Navigator Node Initialized")

    def callback_distance(self, msg):
        """
        Callback function to process incoming ultrasonic distance readings.
        """
        self.current_distance = msg.data
        rospy.loginfo("Current distance: %.2f cm", self.current_distance)

        # Threshold distance for obstacle detection
        if self.current_distance < 20.0:
            rospy.logwarn("Obstacle detected! Stopping or turning.")
            self.avoid_obstacle()
        else:
            self.move_forward()

        # Publish the velocity command
        self.velocity_publisher.publish(self.cmd)

    def move_forward(self):
        """
        Move forward with constant linear speed.
        """
        self.cmd.linear.x = 0.2  # Move straight
        self.cmd.angular.z = 0.0

    def avoid_obstacle(self):
        """
        Rotate in place to avoid detected obstacle.
        """
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.5  # Rotate left

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObstacleAvoidanceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
