#!/usr/bin/env python

import rospy
import numpy as np
import opengen as og
import casadi.casadi as cs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class BoidMPC():
    def __init__(self, agent):
        self.num_robots = rospy.get_param('num_robots', 1)
        self.MPC_horizon = rospy.get_param('MPC_horizon', 5)
        self.nu = 2 # number of inputs
        self.ns = 4 # number of states
        self.J = self.num_robots + 1 # max number of neighbors for separation
        self.param_keys = (        # parameters that do not require recompilation
            "dt",
            "sep_penalty",
            "sep_distance",
            "tradeoff",
            "dynamic_tradeoff",
            "discount",
            "u_weight"
        )
        self.np = len(self.param_keys)
        self.nr = self.ns + 2 * self.MPC_horizon * (2 + self.J) + 1

        self.solution = np.zeros(self.nu * self.MPC_horizon)

        self.cmd_vel_msg = Twist()
        topic_name = f'cmd_vel_{agent}'  # Modify topic naming as needed
        self.cmd_vel_pub = rospy.Publisher(topic_name, Twist, queue_size=10)

        self.robot_positions = np.zeros((self.num_robots, 2))  # Create a numpy array for positions
        self.robot_orientations = np.zeros(self.num_robots)  # Create a numpy array for orientations
        # Subscribe to /odom topics for each robot
        for i in range(self.num_robots):
            topic_name = f'/robot_{i}/odom'  # Modify topic naming as needed
            rospy.Subscriber(topic_name, Odometry, self.odometry_callback, i)

    def odometry_callback(self, msg, robot_index):
        x_position = msg.pose.pose.position.x
        y_position = msg.pose.pose.position.y
        # Extract orientation in radians from quaternion
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Update the corresponding robot's position and orientation
        self.robot_positions[robot_index] = (x_position, y_position)
        self.robot_orientations[robot_index] = yaw

    def cmd_vel_publish(self):
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def make_problem(self):
        # optimization parameter vector (trajectories params)
        vecr = cs.SX.sym("r", self.nr + self.np)
        # optimization variable (input sequence)
        vecu = cs.SX.sym("u", self.T * self.nu)
        R = self.vector2reference(vecr)
        P = self.vector2parameters(vecr)
        u = self.vector2input(vecu)
        cost = self.cost(u, R, P)
        # separation constraints
        f_alm, C_alm = self.separation_constraints(u, R, P)
        # state constraints
        fs, Cs = self.state_constraints(u, R, P)
        if fs is not None:
            # TODO: fix this hack and make it more general
            assert isinstance(Cs, og.constraints.Rectangle)
            f_alm = cs.vertcat(f_alm, fs)
            C_alm = og.constraints.Rectangle(C_alm.xmin + Cs.xmin, C_alm.xmax + Cs.xmax)
        # Bound input constraints
        bounds = self.input_bounds(R, P)
        problem = og.builder.Problem(vecu, vecr, cost) \
            .with_aug_lagrangian_constraints(f_alm, C_alm) \
            .with_constraints(bounds)
        if self.map.obstacles:
            f_pm = self.obstacle_constraints(u, R, P)
            problem = problem.with_penalty_constraints(f_pm)
        return problem

    def vector2reference(self, vecr):
        """
        vecr =  [s_0]
              + [... sk_avg ... ]
              + [... p0_j ... , ... ,  ... pτ_j ...]
              + [tradeoff]
        """
        R = dict()
        T = self.MPC_horizon
        J = self.J
        ns = self.ns
        R["s0"] = self.state_class(vecr[:ns]) """"""
        cohesion = [0.0] * T
        alignment = [0.0] * T
        separation = [0.0] * T
        c = ns
        for k in range(T):
            cohesion[k] = (vecr[c], vecr[c + 1])
            alignment[k] = (vecr[c + 2], vecr[c + 3])
            c += 4
        R["cohesion"] = cohesion
        R["alignment"] = alignment
        for k in range(T):
            Pk = [0.0] * J
            for j in range(J):
                pk_j = vecr[c:c + 2]
                c += 2
                Pk[j] = (pk_j[0], pk_j[1])
            separation[k] = Pk
        R["separation"] = separation
        R["tradeoff"] = vecr[c]
        # c += 1
        return R
    
    def vector2parameters(self, vecr):
        vecp = vecr[-self.np:]
        if type(vecr) is list:
            return {param: p for param, p in zip(self.param_keys, vecp)}
        return {param: p for param, p in zip(self.param_keys, vecp.nz)}

    def vector2input(self, vecu):
        nu = self.nu
        u = [0.0] * self.MPC_horizon
        for k in range(self.MPC_horizon):
            u[k] = vecu[k * nu:(k + 1) * nu]
        return u

if __name__ == '__main__':
    try:
        # agent_0 is the leader
        agent_1 = BoidMPC(agent=1)
        agent_2 = BoidMPC(agent=2)
        agent_3 = BoidMPC(agent=3)
        agent_4 = BoidMPC(agent=4)
    except rospy.ROSInterruptException:
        pass
