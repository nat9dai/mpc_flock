#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from matplotlib.patches import Polygon

from Map import Map
from InequalityObstacle import Rectangle, Curve


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

    def odometry_callback(self, msg, robot_index):
        x_position = msg.pose.pose.position.x
        y_position = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.robot_positions[robot_index], self.robot_orientations[robot_index] = (x_position, y_position), yaw

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
        for idx, ((x, y), yaw) in enumerate(zip(self.robot_positions, self.robot_orientations)):
            if (idx == 0): color = 'red'
            elif (idx == 5): color = 'green'
            else: color = 'blue'
            #color = 'red' if idx == 0 else 'blue'
            self.ax.add_patch(self.create_robot_triangle(x, y, yaw, color))

    def run(self):
        plt.ion()  # Enable interactive mode for real-time plotting
        while not rospy.is_shutdown():
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
        #Start
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
        plotter.map.add_obstacle(Rectangle, (4.1, 4.8), (4.6, 5.5))
        """********************"""

        #plotter.publish_obstacles()  # Publish obstacle information only once
        
        # plt.ion()  # Enable interactive mode for real-time plotting
        # figure = plt.figure()
        # plotter.plotObstacles()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
