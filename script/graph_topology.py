#!/usr/bin/env python

# This node publishes the Adjacency matrix

import rospy
import random
import numpy as np
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

"""
            [0, 0, 1, 0, 0, 0],
            [1, 0, 0, 0, 0, 1],
            [1, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 1, 0]

            [0, 0, 1, 0, 0, 0],
            [3, 0, 0, 0, 0, 1],
            [3, 2, 0, 0, 0, 0],
            [0, 2, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [0, 0, 0, 1, 1, 0]

            [0, 0, sim_loss(loss_rate), 0, 0, 0],
            [sim_loss(loss_rate), 0, 0, 0, 0, sim_loss(loss_rate)],
            [sim_loss(loss_rate), sim_loss(loss_rate), 0, 0, 0, 0],
            [0, sim_loss(loss_rate), 0, 0, 0, 0],
            [0, 0, sim_loss(loss_rate), 0, 0, 0],
            [0, 0, 0, sim_loss(loss_rate), sim_loss(loss_rate), 0]
"""

def adjacency_publisher():
    rospy.init_node('graph_topology')
    pub = rospy.Publisher('adjacency_matrix', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(30)  # 1Hz

    loss_rate = 0.3
    
    while not rospy.is_shutdown():
        adjacency_matrix = np.array([
            [0, 0, 1, 0, 0, 0],
            [3, 0, 0, 0, 0, 1],
            [3, 2, 0, 0, 0, 0],
            [0, 2, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [0, 0, 0, 1, 1, 0]

        ])
        
        # Prepare the message
        msg = Int32MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "height"
        msg.layout.dim[0].size = adjacency_matrix.shape[0]
        msg.layout.dim[0].stride = np.prod(adjacency_matrix.shape)
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[1].label = "width"
        msg.layout.dim[1].size = adjacency_matrix.shape[1]
        msg.layout.dim[1].stride = adjacency_matrix.shape[1]
        msg.layout.data_offset = 0
        msg.data = adjacency_matrix.ravel().tolist()
        
        # Publish the message
        pub.publish(msg)
        rate.sleep()


def sim_loss(loss_rate):
    """
    Simulates data loss based on a given loss rate.

    Parameters:
    loss_rate (int): The data loss rate as an integer from 0 to 100.

    Returns:
    int: 0 if data is lost, 1 if data is not lost.
    """
    if not 0 <= loss_rate <= 1.0:
        raise ValueError("Loss rate must be between 0 and 100")

    # Convert loss rate to probability
    probability = loss_rate

    # Simulate data loss using the random module
    # The random() function generates a float number between 0 and 1.
    return 1 if random.random() >= probability else 0

if __name__ == '__main__':
    try:
        adjacency_publisher()
    except rospy.ROSInterruptException:
        pass