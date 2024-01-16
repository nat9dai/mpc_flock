#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <mpc_flock/RobotStateAndGoal.h>

#include <vector>
#include <cmath>
#include <boost/bind.hpp>

#include <random>

class dataPreparation {
private:
    ros::NodeHandle nh_;
    int num_robots_;
    int num_followers_;
    int num_leaders_;
    int robot_id_;
    int T_; // Horizon period
    std::vector<int> adjacency_row_i_;
    std::vector<std::pair<double, double>> robot_positions_;
    std::vector<std::pair<double, double>> robot_velocity_;
    std::vector<double> robot_orientations_;
    std::vector<std::vector<std::tuple<double, double, double>>> robot_traj_;
    std::vector<ros::Subscriber> odom_sub_list_;
    std::vector<ros::Subscriber> traj_sub_list_;
    ros::Subscriber adjacency_sub_;
    ros::Publisher pub_op_data_;

    double x_avg_;
    double y_avg_;
    double theta_avg_;
    int num_state_;

    std::vector<double> px_avg_;
    std::vector<double> py_avg_;
    std::vector<double> ptheta_avg_;

    std::vector<double> last_yaw_;
    std::vector<double> accumulated_yaw_;
    std::vector<bool> first_yaw_reading_; // Flags to check if it's the first reading of yaw

    std::vector<std::vector<double>> last_yaw_traj_;
    std::vector<std::vector<double>> accumulated_yaw_traj_;
    std::vector<std::vector<bool>> first_yaw_reading_traj_;

    int percent_loss_;

public:
    dataPreparation(int argc, char** argv) : nh_() {
        nh_.getParam("num_robots", num_robots_);
        nh_.getParam("num_followers", num_followers_);
        num_leaders_ = num_robots_ - num_followers_;

        num_state_ = 3;
        T_ = 10;

        adjacency_row_i_.resize(num_robots_, 0);
        adjacency_sub_ = nh_.subscribe("adjacency_matrix", 10, &dataPreparation::adjacencyCallback, this);

        std::string id;
        if (argc >= 2){
            id = argv[1];
        }
        else {
            ROS_ERROR("Robot ID argument not provided. Exiting...");
            ros::shutdown();
            return;   
        }
        robot_id_ = std::stoi(id);

        pub_op_data_ = nh_.advertise<mpc_flock::RobotStateAndGoal>("robot_" + id + "/op_data", 10);

        robot_positions_.resize(num_robots_, std::make_pair(0.0, 0.0));
        robot_velocity_.resize(num_robots_, std::make_pair(0.0, 0.0));
        robot_orientations_.resize(num_robots_, 0.0);
        robot_traj_.resize(num_robots_);
        odom_sub_list_.resize(num_robots_);
        traj_sub_list_.resize(num_robots_);
        last_yaw_.resize(num_robots_);
        accumulated_yaw_.resize(num_robots_);
        first_yaw_reading_.resize(num_robots_);
        for (int i = 0; i < num_robots_; i++){
            first_yaw_reading_[i] = true;
        }

        last_yaw_traj_.resize(num_robots_);
        accumulated_yaw_traj_.resize(num_robots_);
        first_yaw_reading_traj_.resize(num_robots_);

        for (int i = 0; i < num_robots_; i++){
            robot_traj_[i].resize(T_);
            last_yaw_traj_[i].resize(T_);
            accumulated_yaw_traj_[i].resize(T_);
            first_yaw_reading_traj_[i].resize(T_);
        }
        for (int i = 0; i < num_robots_; i++){
            for (int j = 0; j < T_; j++){
                first_yaw_reading_traj_[i][j] = true;
            }
        }
        px_avg_.resize(T_);
        py_avg_.resize(T_);
        ptheta_avg_.resize(T_);

        // loss
        nh_.getParam("data_loss", percent_loss_);

        // Test: subscribe to odom msg from all agents
        for (int i = 0; i< num_robots_;i++){
            std::string topic_name = "/robot_" + std::to_string(i) + "/odom";
            odom_sub_list_[i] = nh_.subscribe<nav_msgs::Odometry>(topic_name, 10, boost::bind(&dataPreparation::odomCallback, this, _1, i));
        }
        for (int i = 0; i< num_robots_;i++){
            std::string topic_name = "/robot_" + std::to_string(i) + "/traj_data";
            traj_sub_list_[i] = nh_.subscribe<nav_msgs::Path>(topic_name, 10, boost::bind(&dataPreparation::trajCallback, this, _1, i));
        }
    }

    void adjacencyCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        int start_index = robot_id_ * num_robots_;
        for (int i = 0; i < num_robots_; ++i) {
            adjacency_row_i_[i] = msg->data[start_index + i];
        }

        /*for (int i = 0; i < num_robots_; ++i) {
            if (adjacency_row_i_[i] != 0 || i == robot_id_) {
                std::string topic_name_odom = "/robot_" + std::to_string(i) + "/odom";
                odom_sub_list_[i] = nh_.subscribe<nav_msgs::Odometry>(topic_name_odom, 10, boost::bind(&dataPreparation::odomCallback, this, _1, i));

                std::string topic_name_traj = "/robot_" + std::to_string(i) + "/traj_data";
                traj_sub_list_[i] = nh_.subscribe<nav_msgs::Path>(topic_name_traj, 10, boost::bind(&dataPreparation::trajCallback, this, _1, i));
            } 
            else {
                if (odom_sub_list_[i]){
                    odom_sub_list_[i].shutdown();
                }
                if (traj_sub_list_[i]){
                    traj_sub_list_[i].shutdown();
                }
            }
        }*/
    }

    void trajCallback(const nav_msgs::Path::ConstPtr& msg, int robot_index){
        for (int i = 0; i < T_; i++){
            if (!msg->poses.empty()){
                double x_position = msg->poses[i].pose.position.x;
                double y_position = msg->poses[i].pose.position.y;
                tf::Quaternion q(msg->poses[i].pose.orientation.x, msg->poses[i].pose.orientation.y, msg->poses[i].pose.orientation.z, msg->poses[i].pose.orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                // If it's the first time reading the yaw, initialize the last_yaw and accumulated_yaw
                if (first_yaw_reading_traj_[robot_index][i]) {
                    last_yaw_traj_[robot_index][i] = yaw;
                    accumulated_yaw_traj_[robot_index][i] = yaw;
                    first_yaw_reading_traj_[robot_index][i] = false;
                } else {
                    // Compute the smallest difference between the new yaw and the last yaw
                    double yaw_diff = yaw - last_yaw_traj_[robot_index][i];
                    // Adjust for overflow (e.g., from pi to -pi)
                    if (yaw_diff > M_PI) {
                        yaw_diff -= 2 * M_PI;
                    } else if (yaw_diff < -M_PI) {
                        yaw_diff += 2 * M_PI;
                    }

                    // Accumulate the yaw difference
                    accumulated_yaw_traj_[robot_index][i] += yaw_diff;

                    // Update the last yaw
                    last_yaw_traj_[robot_index][i] = yaw;
                }

                robot_traj_[robot_index][i] = std::make_tuple(x_position, y_position, accumulated_yaw_traj_[robot_index][i]);                
            }
            else{
                ROS_WARN("Traj msg is empty!");
            }
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int robot_index) {
        double x_position = msg->pose.pose.position.x;
        double y_position = msg->pose.pose.position.y;
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // If it's the first time reading the yaw, initialize the last_yaw and accumulated_yaw
        if (first_yaw_reading_[robot_index]) {
            last_yaw_[robot_index] = yaw;
            accumulated_yaw_[robot_index] = yaw;
            first_yaw_reading_[robot_index] = false;
        } else {
            // Compute the smallest difference between the new yaw and the last yaw
            double yaw_diff = yaw - last_yaw_[robot_index];
            // Adjust for overflow (e.g., from pi to -pi)
            if (yaw_diff > M_PI) {
                yaw_diff -= 2 * M_PI;
            } else if (yaw_diff < -M_PI) {
                yaw_diff += 2 * M_PI;
            }

            // Accumulate the yaw difference
            accumulated_yaw_[robot_index] += yaw_diff;

            // Update the last yaw
            last_yaw_[robot_index] = yaw;
        }

        double v_body = msg->twist.twist.linear.x;
        double x_dot = v_body * cos(accumulated_yaw_[robot_index]);
        double y_dot = v_body * sin(accumulated_yaw_[robot_index]);

        robot_positions_[robot_index] = std::make_pair(x_position, y_position);
        robot_orientations_[robot_index] = accumulated_yaw_[robot_index]; // Store the accumulated yaw
        robot_velocity_[robot_index] = std::make_pair(x_dot, y_dot);
    }

    void pubOptimizationData() {
        int num_of_neighbor = 0;
        x_avg_ = 0.0;
        y_avg_ = 0.0;
        theta_avg_ = 0.0;

        for (int i = 0; i < num_robots_; i++) {
            if ((adjacency_row_i_[i] != 0) || (i == robot_id_)) {
                if (i == robot_id_){
                    x_avg_ += robot_positions_[i].first;
                    y_avg_ += robot_positions_[i].second;
                    theta_avg_ += robot_orientations_[i];
                    num_of_neighbor++;
                }
                else{
                    int not_loss = simulateDataLoss(percent_loss_);
                    x_avg_ += not_loss * adjacency_row_i_[i] * robot_positions_[i].first;
                    y_avg_ += not_loss * adjacency_row_i_[i] * robot_positions_[i].second;
                    theta_avg_ += not_loss * adjacency_row_i_[i] * robot_orientations_[i];
                    num_of_neighbor += not_loss * adjacency_row_i_[i];
                }
            }
        }

        if (num_of_neighbor != 0) {
            x_avg_ /= num_of_neighbor;
            y_avg_ /= num_of_neighbor;
            theta_avg_ = theta_avg_ / num_of_neighbor;
        } else {
            ROS_WARN("No neighbors detected!");
        }

        mpc_flock::RobotStateAndGoal op_data_msg_;
        op_data_msg_.robot_state.resize(num_state_);
        op_data_msg_.goal.resize(num_state_);

        op_data_msg_.robot_state[0] = robot_positions_[robot_id_].first;
        op_data_msg_.robot_state[1] = robot_positions_[robot_id_].second;
        op_data_msg_.robot_state[2] = robot_orientations_[robot_id_];

        op_data_msg_.goal[0] = x_avg_;
        op_data_msg_.goal[1] = y_avg_;
        //op_data_msg_.goal[1] = wrap_to_pi(robot_orientations_[4]-robot_orientations_[robot_id_]);
        op_data_msg_.goal[2] = theta_avg_;
        //op_data_msg_.goal[2] = robot_orientations_[4];

        pub_op_data_.publish(op_data_msg_);
    }

    void pubOptimizationData2() {
        int total_weight = 0;
        for (int i = 0; i < num_robots_; i++){
            if (adjacency_row_i_[i] != 0){
                total_weight += adjacency_row_i_[i];
            }
        }
        for (int i = 0; i < T_; i++){
            px_avg_[i] = 0.0;
            py_avg_[i] = 0.0;
            ptheta_avg_[i] = 0.0;
        }
        mpc_flock::RobotStateAndGoal op_data_msg_;
        op_data_msg_.robot_state.resize(num_state_);
        op_data_msg_.goal.resize(num_state_*T_);
        for (int i = 0; i < T_; i++){
            for (int j = 0; j < num_robots_; j++){
                if (adjacency_row_i_[j] != 0){
                    px_avg_[i] += adjacency_row_i_[j]*std::get<0>(robot_traj_[j][i]);
                    py_avg_[i] += adjacency_row_i_[j]*std::get<1>(robot_traj_[j][i]);
                    ptheta_avg_[i] += adjacency_row_i_[j]*std::get<2>(robot_traj_[j][i]);
                }
            }
            px_avg_[i] += std::get<0>(robot_traj_[robot_id_][i]);
            py_avg_[i] += std::get<1>(robot_traj_[robot_id_][i]);
            ptheta_avg_[i] += std::get<2>(robot_traj_[robot_id_][i]);

            px_avg_[i] /= (total_weight+1);
            py_avg_[i] /= (total_weight+1);
            ptheta_avg_[i] = ptheta_avg_[i] / (total_weight+1);

            int base_index = i * 3;
            op_data_msg_.goal[base_index] = px_avg_[i];
            op_data_msg_.goal[base_index + 1] = py_avg_[i];
            op_data_msg_.goal[base_index + 2] = ptheta_avg_[i];
        }

        //op_data_msg_.robot_state[0] = std::get<0>(robot_traj_[robot_id_][0]);
        //op_data_msg_.robot_state[1] = std::get<1>(robot_traj_[robot_id_][0]);
        //op_data_msg_.robot_state[2] = std::get<2>(robot_traj_[robot_id_][0]);
        op_data_msg_.robot_state[0] = robot_positions_[robot_id_].first;
        op_data_msg_.robot_state[1] = robot_positions_[robot_id_].second;
        op_data_msg_.robot_state[2] = robot_orientations_[robot_id_];

        pub_op_data_.publish(op_data_msg_);
    }

    double wrap_to_pi(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    int simulateDataLoss(int lossRate) {
        if (lossRate < 0 || lossRate > 100) {
            ROS_ERROR("Error: Loss rate must be between 0 and 100");
            return -1; // Return -1 or handle error as appropriate
        }

        // Create a random device and generator
        std::random_device rd;
        std::mt19937 gen(rd());

        // Convert lossRate to probability on the scale 0 to 1
        double probability = lossRate / 100.0;

        // Create Bernoulli distribution with the given probability
        std::bernoulli_distribution d(1.0 - probability);

        // Simulate data loss. The function returns 0 with the probability of lossRate,
        // and 1 with the probability of 1 - lossRate
        return d(gen);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_preparation");
    ros::NodeHandle nh("~");
    dataPreparation data_preparation(argc, argv);
    ros::Rate rate(100);
    while (ros::ok()) {
        data_preparation.pubOptimizationData();
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
