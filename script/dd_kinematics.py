#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math
import sys

class DifferentialDriveKinemetics:
    def __init__(self):
        self.id = sys.argv[1]
        rospy.init_node('dd_kinemetics_{}'.format(self.id))

        self.cmd_vel_sub = rospy.Subscriber('cmd_vel_{}'.format(self.id), Twist, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher('/robot_{}/odom'.format(self.id), Odometry, queue_size=10)

        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link_{}'.format(self.id)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()

        self.rate = rospy.Rate(80)

    def cmd_vel_callback(self, twist_msg):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.last_time).to_sec()

        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z

        delta_theta = angular_velocity * delta_time
        delta_x = linear_velocity * math.cos(self.theta) * delta_time
        delta_y = linear_velocity * math.sin(self.theta) * delta_time

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        quaternion = quaternion_from_euler(0, 0, self.theta)

        self.odom.header.stamp = current_time

        self.odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*quaternion))
        self.odom.twist.twist = Twist(Vector3(linear_velocity, 0, 0), Vector3(0, 0, angular_velocity))

        self.last_time = current_time

    def odom_publish(self):
        self.odom_pub.publish(self.odom)

if __name__ == '__main__':
    try:
        node = DifferentialDriveKinemetics()
        while not rospy.is_shutdown():
            node.odom_publish()
            node.rate.sleep()
    except rospy.ROSInterruptException:
        pass