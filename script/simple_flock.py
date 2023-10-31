#!/usr/bin/env python

import rospy
import numpy as np
import sys
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class ConsensusProtocol:
    def __init__(self):
        self.id = int(sys.argv[1])
        rospy.init_node('simple_flock_{}'.format(self.id))

        self.num_robots = rospy.get_param('num_robots', 1)
        self.num_followers = rospy.get_param('num_followers', 1)
        self.num_leaders = self.num_robots - self.num_followers

        self.adjacency_row_i = np.zeros(self.num_robots)
        self.adjacencySub = rospy.Subscriber('adjacency_matrix', Int32MultiArray, self.adjacencyCallback)

        self.pub_cmd_vel = rospy.Publisher('robot_{}/cmd_vel'.format(self.id), Twist, queue_size=10)
        self.twist_msg = Twist()
        self.Kv = 2.5
        self.Kh = 2.5
        self.delta = 1.0

        self.robot_positions = np.zeros((self.num_robots,2))
        self.robot_orientations = np.zeros(self.num_robots)
        self.odom_sub_list = [None for _ in range(self.num_robots)]

    def adjacencyCallback(self, msg):
        # Convert multiarray to numpy matrix
        self.adjacency_row_i = msg.data[self.id*self.num_robots:self.id*self.num_robots+self.num_robots]
        #self.adjacency_row_i[self.id] = 1
        for i in range(self.num_robots):
            if (self.adjacency_row_i[i] != 0 or i == self.id):
                topic_name = f'/robot_{i}/odom'  # Modify topic naming as needed
                self.odom_sub_list[i] = rospy.Subscriber(topic_name, Odometry, self.odomCallback, i)
            else:
                if self.odom_sub_list[i]:
                    self.odom_sub_list[i].unregister()


    def odomCallback(self, msg, robot_index):
        x_position = msg.pose.pose.position.x
        y_position = msg.pose.pose.position.y
        # Extract orientation in radians from quaternion
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Update the corresponding robot's position and orientation
        self.robot_positions[robot_index] = (x_position, y_position)
        self.robot_orientations[robot_index] = yaw

    def pubControlLaw(self):
        delta_position = np.zeros(2)
        for i in range(self.num_robots):
            if self.adjacency_row_i[i] != 0:
                delta_position += self.adjacency_row_i[i] * (self.robot_positions[i] - self.robot_positions[self.id])

        desired_heading = np.arctan2(delta_position[1], delta_position[0])
        speed = self.Kv * (np.linalg.norm(delta_position) - self.delta)
        angular_speed = self.Kh * self.wrap_to_pi(desired_heading - self.robot_orientations[self.id])

        self.twist_msg.linear.x = speed
        self.twist_msg.angular.z = angular_speed
        self.pub_cmd_vel.publish(self.twist_msg)

    @staticmethod
    def wrap_to_pi(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
        
if __name__ == "__main__":
    try:
        consensus_protocol = ConsensusProtocol()
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            consensus_protocol.pubControlLaw()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass