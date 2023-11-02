#!/usr/bin/env python

# This node publishes the Adjacency matrix

import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

"""
            [0, 0, 0, 0, 0, 0],
            [2, 0, 0, 0, 0, 1],
            [2, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 1, 0]
"""

def adjacency_publisher():
    rospy.init_node('graph_topology')
    pub = rospy.Publisher('adjacency_matrix', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 1Hz
    
    while not rospy.is_shutdown():
        # Sample 3x3 adjacency matrix for demonstration
        # Adjust size and values as required for your multi-agent system
        adjacency_matrix = np.array([
            [0, 0, 0, 0, 0, 0],
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

if __name__ == '__main__':
    try:
        adjacency_publisher()
    except rospy.ROSInterruptException:
        pass