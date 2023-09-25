#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from matplotlib.patches import Polygon
from geometry_msgs.msg import PolygonStamped, Point32

class RobotPlotter:
    def __init__(self):
        self.num_robots = rospy.get_param('num_robots', 1)
        self.robot_positions = np.zeros((self.num_robots, 2))  # Create a numpy array for positions
        self.robot_orientations = np.zeros(self.num_robots)  # Create a numpy array for orientations
        self.robot_triangles = [None] * self.num_robots  # Initialize a list for the robot triangles

        # Define the plot limits (adjust as needed)
        self.xlim_min = rospy.get_param('xlim_min', -5.0)
        self.xlim_max = rospy.get_param('xlim_max',  5.0)
        self.ylim_min = rospy.get_param('ylim_min', -5.0)
        self.ylim_max = rospy.get_param('ylim_max',  5.0)

        rospy.init_node('planar_simulation')

        # Create a publisher for obstacles (latched)
        self.obstacle_pub = rospy.Publisher('/obstacles', PolygonStamped, queue_size=1, latch=True)

        # Subscribe to /odom topics for each robot
        for i in range(self.num_robots):
            topic_name = f'/robot_{i}/odom'  # Modify topic naming as needed
            rospy.Subscriber(topic_name, Odometry, self.odometry_callback, i)

        # Set up the initial plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(self.xlim_min, self.xlim_max)
        self.ax.set_ylim(self.ylim_min, self.ylim_max)

        self.rate = rospy.Rate(30)  # Update rate in Hz

        # Define the obstacle polygons (added here for static obstacles)
        self.obstacle_polygons = [
            [(1.0, 2.0), (1.0, 3.0), (2.0, 3.0), (2.0, 2.0)],
            [(-3.0, -2.0), (-3.0, -1.0), (-4.0, -1.0), (-4.0, -2.0)]
        ]

    def odometry_callback(self, msg, robot_index):
        x_position = msg.pose.pose.position.x
        y_position = msg.pose.pose.position.y
        # Extract orientation in radians from quaternion
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Update the corresponding robot's position and orientation
        self.robot_positions[robot_index] = (x_position, y_position)
        self.robot_orientations[robot_index] = yaw

    def create_robot_triangle(self, x, y, yaw):
        # Define the vertices of the triangle
        side_length = 0.4
        front_length = 0.5
        half_length = side_length / 2
        vertices = [
            (x - half_length, y - half_length),
            (x - half_length, y + half_length),
            (x + front_length, y)
        ]

        # Rotate the triangle based on yaw
        rot_matrix = np.array([[np.cos(yaw), -np.sin(yaw)],
                                [np.sin(yaw), np.cos(yaw)]])
        rotated_vertices = np.dot(vertices - np.array([x, y]), rot_matrix.T) + np.array([x, y])

        return Polygon(rotated_vertices, closed=True, edgecolor='k', facecolor='blue')

    def publish_obstacles(self):
        # Create and publish obstacle polygons
        obstacle_msg = PolygonStamped()
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = 'map'  # Change 'map' to your desired frame ID

        for obstacle_polygon in self.obstacle_polygons:
            obstacle_msg.polygon.points.extend([Point32(x=pt[0], y=pt[1]) for pt in obstacle_polygon])

        self.obstacle_pub.publish(obstacle_msg)

    def update_plot(self):
        self.ax.clear()  # Clear the axes

        # Set the plot limits
        self.ax.set_xlim(self.xlim_min, self.xlim_max)
        self.ax.set_ylim(self.ylim_min, self.ylim_max)

        self.ax.set_title('Robot Positions and Orientations')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')

        # Plot obstacles
        for obstacle_polygon in self.obstacle_polygons:
            obstacle_patch = Polygon(obstacle_polygon, closed=True, edgecolor='k', facecolor='gray')
            self.ax.add_patch(obstacle_patch)

        # Plot each robot's position and orientation
        for idx, ((x, y), yaw) in enumerate(zip(self.robot_positions, self.robot_orientations)):
            if idx == 0:
                color = 'red'  # Use a different color (e.g., red) for the first robot
            else:
                color = 'blue'

            if self.robot_triangles[idx] is None:
                robot_triangle = self.create_robot_triangle(x, y, yaw)
                robot_triangle.set_facecolor(color)  # Set the face color
                self.robot_triangles[idx] = robot_triangle
                self.ax.add_patch(robot_triangle)
            else:
                robot_triangle = self.create_robot_triangle(x, y, yaw)
                robot_triangle.set_facecolor(color)  # Set the face color
                self.ax.add_patch(robot_triangle)

    def run(self):
        while not rospy.is_shutdown():
            self.update_plot()
            plt.pause(0.03)  # Pause for a short time to allow the plot to update
            self.rate.sleep()

if __name__ == '__main__':
    try:
        plotter = RobotPlotter()
        plotter.publish_obstacles()  # Publish obstacle information only once
        plt.ion()  # Enable interactive mode for real-time plotting
        plotter.run()
    except rospy.ROSInterruptException:
        pass
