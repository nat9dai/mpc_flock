import casadi as cs
import opengen as og
import numpy as np

N = 10  # The MPC horizon length
NX = 3  # The number of elements in the state vector
NU = 2  # The number of elements in the control vector
sampling_time = 0.1

Q = cs.DM.eye(NX) * [5.0, 5.0, 1.0]
R = cs.DM.eye(NU) * [0.1, 0.01]
QN = cs.DM.eye(NX) * [10.0, 10.0, 5.0]

# return x_dot
def dynamics_ct(_x, _u):
    return cs.vcat([_u[0] * cs.cos(_x[2]),
                    _u[0] * cs.sin(_x[2]),
                    _u[1]])

# update state
def dynamics_dt(x, u):
    dx = dynamics_ct(x, u)
    return cs.vcat([x[i] + sampling_time * dx[i] for i in range(NX)])


def wrap_to_pi(angle):
    """ Wrap angle to [-pi, pi] using CasADi. """
    return cs.fmod(angle + cs.pi, 2 * cs.pi) - cs.pi


# The stage cost for x and u
def stage_cost(_x, _u, _x_ref=None, _u_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    if _u_ref is None:
        _u_ref = cs.DM.zeros(_u.shape)

    # Use wrap_to_pi to ensure heading difference is in the range [-pi, pi]
    dtheta = _x[2] - _x_ref[2]

    # Compute the state difference for position and wrapped heading
    dx = cs.vertcat(_x[0] - _x_ref[0], _x[1] - _x_ref[1], dtheta)

    # Compute the control input difference
    du = _u - _u_ref

    # Calculate the stage cost
    return cs.mtimes([dx.T, Q, dx]) + cs.mtimes([du.T, R, du])

# The terminal cost for x
def terminal_cost(_x, _x_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)

    dtheta = _x[2] - _x_ref[2]
    dx = cs.vertcat(_x[0] - _x_ref[0], _x[1] - _x_ref[1], dtheta)
    return cs.mtimes([dx.T, QN, dx])
#x_0 = cs.MX.sym("x_0", NX)
x_t = [cs.MX.sym('x_' + str(i), NX) for i in range(N)]
x_ref = [cs.MX.sym('x_ref_' + str(i), NX) for i in range(N)]
u_k = [cs.MX.sym('u_' + str(i), NU) for i in range(N)]

# Create the cost function
total_cost = 0

for t in range(0, N-1):
    total_cost += stage_cost(x_t[t], u_k[t], x_ref[t])  # update cost
    x_t[t+1] = dynamics_dt(x_t[t], u_k[t])  # update state

total_cost += terminal_cost(x_t[-1], x_ref[-1])  # terminal cost

optimization_variables = []
optimization_parameters = []

optimization_variables += u_k
#optimization_variables += x_t[1:]
optimization_parameters += [x_t[0]]
#optimization_parameters += [x_t[i] for i in range(N)] # ERROR!
optimization_parameters += x_ref
#print(optimization_parameters)
#print(len(u_k))
#exit()

optimization_variables = cs.vertcat(*optimization_variables)
optimization_parameters = cs.vertcat(*optimization_parameters)

umin = [-2.0, -10.0] * N  # - cs.DM.ones(NU * N) * cs.inf
umax = [2.0, 10.0] * N  # cs.DM.ones(NU * N) * cs.inf

bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(optimization_variables,
                             optimization_parameters,
                             total_cost) \
    .with_constraints(bounds)

ros_config = og.config.RosConfiguration() \
    .with_package_name("distributed_nmpc_controller") \
    .with_node_name("distributed_nmpc_controller_node") \
    .with_rate((int)(1.0/sampling_time)) \
    .with_description("ROS ndoe for distributed nmpc")

build_config = og.config.BuildConfiguration()\
    .with_build_directory("optimization_engine")\
    .with_build_mode("release")\
    .with_build_c_bindings() \
    .with_ros(ros_config)

meta = og.config.OptimizerMeta()\
    .with_optimizer_name("mpc_controller")

solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()
