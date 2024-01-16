#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import time

# Global dictionary to store positions and timestamps
data = {'x': {}, 'y': {}, 'time': {}}

def odom_callback(msg, args):
    """ Callback function to process Odometry data. """
    robot_id = args
    current_time = time.time() - start_time
    data['time'].setdefault(robot_id, []).append(current_time)
    data['x'].setdefault(robot_id, []).append(msg.pose.pose.position.x)
    data['y'].setdefault(robot_id, []).append(msg.pose.pose.position.y)

def plot_positions(data):
    """ Plot the positions of the robots over time. """
    colors = ['r', 'g', 'b', 'y', 'c', 'm']  # Different colors for each robot

    plt.figure()
    for i, robot_id in enumerate(data['x']):
        plt.subplot(2, 1, 1)
        plt.plot(data['time'][robot_id], data['x'][robot_id], color=colors[robot_id], label=f'agent {robot_id}')
        plt.ylabel('X Position')
        plt.title(f'Unicycle Distributed NMPC: Data Loss = {percent_loss}%')
        plt.legend()
        plt.grid(True)

        plt.subplot(2, 1, 2)
        plt.plot(data['time'][robot_id], data['y'][robot_id], color=colors[robot_id], label=f'agent {robot_id}')
        plt.xlabel('Time (s)')
        plt.ylabel('Y Position')
        plt.legend()
        plt.grid(True)

    plt.show()

def main():
    global start_time
    start_time = time.time()
    rospy.init_node('robot_position_plotter', anonymous=True)

    global percent_loss
    percent_loss = rospy.get_param('data_loss', 0)
    plot_time = rospy.get_param('plot_time', 10)

    # Setup subscribers for each robot
    num_robots = rospy.get_param('num_robots', 1)
    for i in range(num_robots):
        rospy.Subscriber(f'/robot_{i}/odom', Odometry, odom_callback, i)

    rospy.sleep(plot_time)  # Collect data for 10 seconds

    plot_positions(data)

if __name__ == '__main__':
    main()
