import casadi as cs
import opengen as og

# NMPC code generator for the unicycle model

T = 10  # Horizon period
NX = 3  # The number of states (x, y, theta)
NU = 2  # The number of control variables (v, w)
sampling_time = 0.1
NO = 50 # No. of obstacles

Q = cs.DM.eye(NX) * [1000.0, 1000.0, 0.1]
R = cs.DM.eye(NU) * [0.1, 0.1]
QT = cs.DM.eye(NX) * [2000.0, 2000.0, 0.2]
#QT = cs.DM.eye(NX) * [100000.0, 100000.0, 1000.0]


def dynamics_ct(_x, _u):
    return cs.vcat([_u[0] * cs.cos(_x[2]),
                    _u[0] * cs.sin(_x[2]),
                    _u[1]])


def dynamics_dt(x, u):
    dx = dynamics_ct(x, u)
    return cs.vcat([x[i] + sampling_time * dx[i] for i in range(NX)])


# The stage cost for x and u
def stage_cost(_x, _u, _x_ref=None, _u_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    if _u_ref is None:
        _u_ref = cs.DM.zeros(_u.shape)
    dx = _x - _x_ref
    du = _u - _u_ref
    return cs.mtimes([dx.T, Q, dx]) + cs.mtimes([du.T, R, du])
    #return cs.mtimes([du.T, R, du])



# The terminal cost
def terminal_cost(_x, _x_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    dx = _x - _x_ref
    return cs.mtimes([dx.T, QT, dx])

def obstacle_constraints(x_0, u, obs, T):
    f2 = []
    min_distance = 0.5

    x = x_0

    for k in range(T):
        x = dynamics_dt(x, u[k])
        for o in obs:
            dx = x[0] - o[0]
            dy = x[1] - o[1]
            distance = cs.sqrt(dx**2 + dy**2)
            violation = min_distance**2 - distance
            f2 = cs.vertcat(f2, cs.fmax(0, violation))

    return f2

def obstacle_avoidance_penalty(x, obs, min_distance):
    penalty = 0
    for o in obs:
        squared_dist = (x[0] - o[0])**2 + (x[1] - o[1])**2
        # Increase penalty as the robot gets closer to the obstacle
        penalty += cs.exp(min_distance**2 - squared_dist)
        #penalty += cs.fmax(0, min_distance**2 - squared_dist)
    return penalty

x_0 = cs.MX.sym("x_0", NX)
x_ref = cs.MX.sym("x_ref", NX)
u_k = [cs.MX.sym('u_' + str(i), NU) for i in range(T)]
obs_i = [cs.MX.sym('obs_' + str(i), 2) for i in range(NO)]

# Create the cost function
x_t = x_0
total_cost = 0
obstacle_penalty_weight = 10000
min_distance = 0.5

for t in range(0, T):
    total_cost += stage_cost(x_t, u_k[t], x_ref)  # update cost
    #total_cost += obstacle_penalty_weight * obstacle_avoidance_penalty(x_t, obs_i, min_distance)
    x_t = dynamics_dt(x_t, u_k[t])  # update state

total_cost += terminal_cost(x_t, x_ref)  # terminal cost
#total_cost += obstacle_penalty_weight*obstacle_avoidance_penalty(x_t, obs_i, min_distance)

optimization_variables = []
optimization_parameters = []

optimization_variables += u_k
optimization_parameters += [x_0]
optimization_parameters += [x_ref]
optimization_parameters += obs_i

optimization_variables = cs.vertcat(*optimization_variables)
optimization_parameters = cs.vertcat(*optimization_parameters)

umin = [-1.0, -10.0] * T  # - cs.DM.ones(NU * T) * cs.inf
umax = [1.0, 10.0] * T  # cs.DM.ones(NU * T) * cs.inf

f_pm = obstacle_constraints(x_0, u_k, obs_i, T)

bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(optimization_variables,
                             optimization_parameters,
                             total_cost) \
    .with_penalty_constraints(f_pm) \
    .with_constraints(bounds)

#problem = problem.with_penalty_constraints(f_pm)

ros_config = og.config.RosConfiguration() \
    .with_package_name("simple_distributed_nmpc_controller") \
    .with_node_name("simple_distributed_nmpc_controller_node") \
    .with_rate((int)(1.0/sampling_time)) \
    .with_description("ROS node for simple nmpc controller.")

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