#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import sys

robot_id = sys.argv[1]

# Initialize the ROS node
rospy.init_node('cmd_vel_keyboard')

# Create a publisher for the cmd_vel topic
pub = rospy.Publisher('/robot_{}/cmd_vel'.format(robot_id), Twist, queue_size=10)

# Set the publishing rate to 60 Hz
rate = rospy.Rate(100)

# Initialize the Twist message
twist_msg = Twist()

# Define the key mapping for arrow keys
key_mapping = {
    keyboard.Key.up: (2.0, 0.0),
    keyboard.Key.down: (-2.0, 0.0),
    keyboard.Key.left: (0.0, 1.0),
    keyboard.Key.right: (0.0, -1.0)
}

# Initialize the velocity values
linear_vel = 0.0
angular_vel = 0.0

# Maintain a set of currently pressed keys
pressed_keys = set()

def update_twist():
    global linear_vel, angular_vel

    # Calculate velocities based on the currently pressed keys
    linear_vel = sum(key_mapping[key][0] for key in pressed_keys)
    angular_vel = sum(key_mapping[key][1] for key in pressed_keys)

    twist_msg.linear.x = linear_vel
    twist_msg.angular.z = angular_vel
    pub.publish(twist_msg)

def on_press(key):
    if key in key_mapping:
        pressed_keys.add(key)
        update_twist()

def on_release(key):
    if key in key_mapping:
        pressed_keys.remove(key)
        update_twist()

# Create a listener for keyboard input
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    while not rospy.is_shutdown():
        update_twist()
        rate.sleep()