#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from matplotlib.patches import Polygon

from Map import Map
from InequalityObstacle import Rectangle, Curve

import random
from std_msgs.msg import Float32MultiArray


class RobotPlotter:

    def __init__(self):
        self.num_robots = rospy.get_param('num_robots', 1)
        self.robot_positions = np.zeros((self.num_robots, 2))
        self.robot_orientations = np.zeros(self.num_robots)

        # Define plot limits
        self.xlim_min, self.xlim_max = rospy.get_param('xlim_min', -5.0), rospy.get_param('xlim_max', 5.0)
        self.ylim_min, self.ylim_max = rospy.get_param('ylim_min', -5.0), rospy.get_param('ylim_max', 5.0)

        rospy.init_node('planar_simulation')

        for i in range(self.num_robots):
            rospy.Subscriber(f'/robot_{i}/odom', Odometry, self.odometry_callback, i)

        # Setup initial plot
        self.fig, self.ax = plt.subplots()
        self.rate = rospy.Rate(60)
        self.map = Map()

        self.num_obstacle = 50
        self.array_obstacle = np.zeros((self.num_obstacle, 2))
        self.random_obstracle()
        #print(self.array_obstacle)
        self.obs_pub = rospy.Publisher('obstacle_array', Float32MultiArray, queue_size=10)
        self.obs_msg = Float32MultiArray()
        self.obs_msg.data = self.array_obstacle.flatten()

        # Set up for making GIF
        self.frame_counter = 0

    def odometry_callback(self, msg, robot_index):
        x_position = msg.pose.pose.position.x
        y_position = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.robot_positions[robot_index], self.robot_orientations[robot_index] = (x_position, y_position), yaw

    def random_obstracle(self):
        """obs_x = [
                2.4658610820770264, 4.109138488769531, 1.5227540731430054, -4.45414400100708, 
                -3.384025812149048, 4.589311599731445, 0.5373553037643433, -4.489222049713135, 
                -2.050752639770508, 3.727564573287964, -3.534968137741089, -2.2837984561920166, 
                 0.9086710214614868, -4.354544162750244, 0.6323286294937134, 0.8622678518295288, 
                -3.060565710067749, 0.6250110268592834, -0.4955260455608368, -4.092363357543945
                ]
        obs_y = [
                4.017983436584473, 4.078245639801025, -2.4628186225891113, 4.844747066497803, 
                -3.525289297103882, 1.531572699546814, 2.523827075958252, 2.0723724365234375, 
                4.53257942199707, -2.622781276702881, -1.2919397354125977, -1.6671830415725708, 
                -0.5196019411087036, 4.054372787475586, 1.5908114910125732, -2.0674476623535156, 
                -0.4441092908382416, 0.5658187866210938, 1.9592065811157227, 1.0359582901000977
                ]"""
        for i in range(self.num_obstacle):
            obs_x = random.uniform(self.xlim_min, self.xlim_max)
            obs_y = random.uniform(self.ylim_min, self.ylim_max)
            self.array_obstacle[i][0] = obs_x
            self.array_obstacle[i][1] = obs_y


    @staticmethod
    def create_robot_triangle(x, y, yaw, color='blue'):
        side_length, front_length = 0.3, 0.45
        half_length = side_length / 2
        vertices = [
            (x - half_length, y - half_length),
            (x - half_length, y + half_length),
            (x + front_length, y)
        ]
        rot_matrix = np.array([[np.cos(yaw), -np.sin(yaw)],
                               [np.sin(yaw), np.cos(yaw)]])
        rotated_vertices = np.dot(vertices - np.array([x, y]), rot_matrix.T) + np.array([x, y])
        return Polygon(rotated_vertices, closed=True, edgecolor='k', facecolor=color)

    def update_plot(self):
        self.ax.clear()
        self.ax.set(xlim=(self.xlim_min, self.xlim_max), ylim=(self.ylim_min, self.ylim_max),
                    title='Robot Positions and Orientations', xlabel='X Position (m)', ylabel='Y Position (m)')
        
        #plot obstacles
        obs_xs = self.array_obstacle[:, 0]
        obs_ys = self.array_obstacle[:, 1]
        self.ax.plot(obs_xs, obs_ys, 'o')

        for idx, ((x, y), yaw) in enumerate(zip(self.robot_positions, self.robot_orientations)):
            #if (idx == 0): color = 'red'
            #elif (idx == 5): color = 'green'
            #else: color = 'blue'
            color = 'blue'
            self.ax.add_patch(self.create_robot_triangle(x, y, yaw, color))

        # Save the current frame as an image file
        plt.savefig(f'/home/nat/Desktop/buffer/frame_{self.frame_counter}.png')
        self.frame_counter += 1

    def run(self):
        plt.ion()  # Enable interactive mode for real-time plotting
        #while not rospy.is_shutdown():
        while not rospy.is_shutdown() and self.frame_counter < 300:
            self.obs_pub.publish(self.obs_msg)
            self.update_plot()
            plt.pause(0.01)
            self.rate.sleep()

    def plot_obstacles(self):
        for obs in self.map.obstacles:
            obs_x, obs_y = obs.plot()
            self.ax.plot(obs_x, obs_y, "k", linewidth=2)
            if hasattr(obs, 'is_enlarged') and obs.is_enlarged:
                obs.enlarged.plot("k--", linewidth=0.5)

    def publish_obstacles(self):
        # Create and publish obstacle polygons
        pass
        '''
        obstacle_msg = PolygonStamped()
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = 'map'  # Change 'map' to your desired frame ID

        for obstacle_polygon in self.obstacle_polygons:
            obstacle_msg.polygon.points.extend([Point32(x=pt[0], y=pt[1]) for pt in obstacle_polygon])

        self.obstacle_pub.publish(obstacle_msg)'''

if __name__ == '__main__':
    try:
        plotter = RobotPlotter()

        """ Draw the obstacles here! """
        """#Start
        plotter.map.add_obstacle(Rectangle, (-7, 4), (-5, 14))
        plotter.map.add_obstacle(Rectangle, (-3.5, 9), (0, 14))

        # First curve
        plotter.map.add_obstacle(Curve, center=(0, 9), inner=3.5, outer=5, angle=180)

        # working earlier
        plotter.map.add_obstacle(Rectangle, (0, 5.5), (11, 9))
        plotter.map.add_obstacle(Rectangle, (-7, 4), (6, 0.5))

        # Second curve
        plotter.map.add_obstacle(Curve, center=(6, 0.5), inner=3.5, outer=5, angle=0)

        plotter.map.add_obstacle(Rectangle, (0.5, 4), (1, 4.9))
        plotter.map.add_obstacle(Rectangle, (4.1, 4.8), (4.6, 5.5))"""
        """********************"""

        #plotter.publish_obstacles()  # Publish obstacle information only once
        
        # plt.ion()  # Enable interactive mode for real-time plotting
        # figure = plt.figure()
        # plotter.plotObstacles()
        plotter.run()
    except rospy.ROSInterruptException:
        pass