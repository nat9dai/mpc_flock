/**
 * This is an auto-generated file by Optimization Engine (OpEn)
 * OpEn is a free open-source software - see doc.optimization-engine.xyz
 * dually licensed under the MIT and Apache v2 licences.
 *
 */
#include "ros/ros.h"
#include "simple_distributed_nmpc_controller/OptimizationResult.h"
#include "simple_distributed_nmpc_controller/OptimizationParameters.h"
#include "mpc_controller_bindings.hpp"
#include "open_optimizer.hpp"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <mpc_flock/RobotStateAndGoal.h>

namespace simple_distributed_nmpc_controller {
/**
 * Class simple_distributed_nmpc_controller::OptimizationEngineManager manages the
 * exchange of data between the input and output topics
 * of this node
 */
class OptimizationEngineManager {
/**
 * Private fields and methods
 */
private:
    static const int NX = 3;
    static const int NU = 2;
    static const int NO = 50;
    
    double current_pos[NX] = {0};
    double current_ref[NX] = {0};
    double obstacles[2*NO] = {0};

    /**
     * Optimization parameters announced on the corresponding
     * topic (simple_distributed_nmpc_controller/parameters)
     */
    simple_distributed_nmpc_controller::OptimizationParameters params;
    /**
     * Object containing the result (solution and solver
     * statistics), which will be announced on simple_distributed_nmpc_controller/results
     */
    simple_distributed_nmpc_controller::OptimizationResult results;
    /**
     * Vector of parameters (provided by the client on
     * simple_distributed_nmpc_controller/parameters)
     */
    double p[MPC_CONTROLLER_NUM_PARAMETERS] = { 0 };
    /**
     * Solution vector
     */
    double u[MPC_CONTROLLER_NUM_DECISION_VARIABLES] = { 0 };
    /**
     * Vector of Lagrange multipliers (if any)
     */
    double *y = NULL;
    /**
     * Workspace variable used by the solver - initialised once
     */
    mpc_controllerCache* cache;
    /**
     * Initial guess for the penalty parameter
     */
    double init_penalty = ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY;

    /**
     * Publish obtained results to output topic
     */
    void publishToTopic(ros::Publisher& publisher)
    {
        publisher.publish(results);
    }

    /**
     * Updates the input data based on the data that are posted
     * on /mpc/open_parameters (copies value from topic data to
     * local variables). This method is responsible for parsing
     * the data announced on the input topic.
     */
    void updateInputData()
    {
        init_penalty = (params.initial_penalty > 1.0)
            ? params.initial_penalty
            : ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY;

        if (params.parameter.size() > 0) {
            for (size_t i = 0; i < MPC_CONTROLLER_NUM_PARAMETERS; ++i)
                p[i] = params.parameter[i];
        }

        if (params.initial_guess.size() == MPC_CONTROLLER_NUM_DECISION_VARIABLES) {
            for (size_t i = 0; i < MPC_CONTROLLER_NUM_DECISION_VARIABLES; ++i)
                u[i] = params.initial_guess[i];
        }

		if (params.initial_y.size() == MPC_CONTROLLER_N1) {
            for (size_t i = 0; i < MPC_CONTROLLER_N1; ++i)
                y[i] = params.initial_y[i];
		}
    }

    /**
     * Call OpEn to solve the problem
     */
    mpc_controllerSolverStatus solve()
    {
        return mpc_controller_solve(cache, u, p, y, &init_penalty);
    }
/**
 * Public fields and methods
 */
public:
    /**
     * Constructor of OptimizationEngineManager
     */
    OptimizationEngineManager()
    {
	    y = new double[MPC_CONTROLLER_N1];
        cache = mpc_controller_new();
    }

    /**
     * Destructor of OptimizationEngineManager
     */
    ~OptimizationEngineManager()
    {
		if (y!=NULL) delete[] y;
        mpc_controller_free(cache);
    }

    /**
     * Copies results from `status` to the local field `results`
     */
    void updateResults(mpc_controllerSolverStatus& status)
    {
        std::vector<double> sol(u, u + MPC_CONTROLLER_NUM_DECISION_VARIABLES);
        results.solution = sol;
        std::vector<double> y(status.lagrange, status.lagrange + MPC_CONTROLLER_N1);
        results.lagrange_multipliers = y;
        results.inner_iterations = status.num_inner_iterations;
        results.outer_iterations = status.num_outer_iterations;
        results.norm_fpr = status.last_problem_norm_fpr;
        results.cost = status.cost;
        results.penalty = status.penalty;
        results.status = (int)status.exit_status;
        results.solve_time_ms = (double)status.solve_time_ns / 1000000.0;
        results.infeasibility_f2 = status.f2_norm;
        results.infeasibility_f1 = status.delta_y_norm_over_c;
    }

    /**
     * Callback that obtains data from topic `/simple_distributed_nmpc_controller/open_params`
     */
    void mpcReceiveRequestCallback(
        const simple_distributed_nmpc_controller::OptimizationParameters::ConstPtr& msg)
    {
        params = *msg;
    }

    void solveAndPublish(ros::Publisher& publisher)
    {
        updateInputData(); /* get input data */
        mpc_controllerSolverStatus status = solve(); /* solve!  */
        updateResults(status); /* pack results into `results` */
        publishToTopic(publisher);
    }

    void dataCallback(const mpc_flock::RobotStateAndGoalConstPtr& msg){
        for (int i = 0; i < NX; i++){
            current_pos[i] = msg->robot_state[i];
            current_ref[i] = msg->goal[i];
        }
        
    }

    void obsCallback(const std_msgs::Float32MultiArrayConstPtr& msg){
        for (int i = 0; i < 2*NO; i++){
            obstacles[i] = msg->data[i];
        }
    }

    void solveAndPublishCmdVel(ros::Publisher& publisher)
    {
        double current_par [MPC_CONTROLLER_NUM_PARAMETERS] = {0};
        double current_var [MPC_CONTROLLER_NUM_DECISION_VARIABLES] = {0};
        double lin_vel_cmd, ang_vel_cmd = 0;
        //ROS_INFO("No: %d",MPC_CONTROLLER_NUM_PARAMETERS);


        for (int i=0; i<NX; i++) {
            current_par[i] = current_pos[i];
            current_par[i+NX] = current_ref[i];
        }

        int j = 0;
        for (int i=2*NX; i<2*(NX+NO); i++){
            current_par[i] = obstacles[j];
            j++;
            //ROS_INFO("j: %d", j);
        }

        //ROS_INFO("x: %f, y: %f", current_par[25], current_par[26]);
        
        /* solve                  */
        mpc_controllerSolverStatus status
            = mpc_controller_solve(cache, current_var, current_par, 0, &init_penalty);
        
        lin_vel_cmd = current_var[0];
        ang_vel_cmd = current_var[1];
    
        geometry_msgs::Twist twist;
        twist.linear.x = lin_vel_cmd;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = ang_vel_cmd;
        publisher.publish(twist);

        //ROS_INFO("v: %f, w: %f", current_var[18], current_var[19]);
        
        //ROS_INFO("x: %f, y: %f, yaw: %f", current_pos[0], current_pos[1], current_pos[2]);
        //ROS_INFO("Solve time: %f ms. I will send %f %f \n",
        //         (double)status.solve_time_ns / 1000000.0, lin_vel_cmd, ang_vel_cmd);

    }
}; /* end of class OptimizationEngineManager */
} /* end of namespace simple_distributed_nmpc_controller */

/**
 * Main method
 *
 * This advertises a new (private) topic to which the optimizer
 * announces its solution and solution status and details. The
 * publisher topic is 'simple_distributed_nmpc_controller/result'.
 *
 * It obtains inputs from 'simple_distributed_nmpc_controller/parameters'.
 *
 */
int main(int argc, char** argv)
{
    std::string result_topic, params_topic;  /* parameter and result topics */
    double rate; /* rate of node (specified by parameter) */

    simple_distributed_nmpc_controller::OptimizationEngineManager mng;
    ros::init(argc, argv, ROS_NODE_MPC_CONTROLLER_NODE_NAME);
    ros::NodeHandle nh, private_nh("~");

    /* obtain parameters from config/open_params.yaml file */
    private_nh.param("result_topic", result_topic,
                     std::string(ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC));
    private_nh.param("params_topic", params_topic,
                     std::string(ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC));
    //private_nh.param("rate", rate,
    //                 double(ROS_NODE_MPC_CONTROLLER_RATE));
    rate = 100;

    ros::Publisher mpc_pub
        = private_nh.advertise<simple_distributed_nmpc_controller::OptimizationResult>(
            result_topic,
            ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC_QUEUE_SIZE);
    ros::Subscriber sub
        = private_nh.subscribe(
            params_topic,
            ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC_QUEUE_SIZE,
            &simple_distributed_nmpc_controller::OptimizationEngineManager::mpcReceiveRequestCallback,
            &mng);
    ros::Rate loop_rate(rate);

    std::string id;
    int robot_id;
    if (argc >= 2){
        id = argv[1];
    }
    else {
        ROS_ERROR("Robot ID argument not provided. Exiting...");
        ros::shutdown();
        return 1;   
    }

    ros::Subscriber optimization_data_sub
        = nh.subscribe("robot_" + id +"/op_data",
                               10,
                               &simple_distributed_nmpc_controller::OptimizationEngineManager::dataCallback,
                               &mng);
    ros::Subscriber obs_sub
        = nh.subscribe("obstacle_array",
                               10,
                               &simple_distributed_nmpc_controller::OptimizationEngineManager::obsCallback,
                               &mng);
    ros::Publisher pub_twist_cmd = nh.advertise<geometry_msgs::Twist>("robot_" + id + "/cmd_vel", 10);

    while (ros::ok()) {
        mng.solveAndPublishCmdVel(pub_twist_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}