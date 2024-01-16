#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cmath>
#include <boost/bind.hpp>

#include <random>

class ConsensusProtocol {
private:
    ros::NodeHandle nh_;
    int num_robots_;
    int num_followers_;
    int num_leaders_;
    int robot_id_;
    std::vector<int> adjacency_row_i_;
    std::vector<std::pair<double, double>> robot_positions_;
    std::vector<std::pair<double, double>> robot_velocity_;
    std::vector<double> robot_orientations_;
    std::vector<ros::Subscriber> odom_sub_list_;
    double Kp_;
    double Kv_;
    double Khp_;
    double Khv_;
    double Khi_;
    double delta_;

    double Kalpha_;
    double Kbeta_;

    ros::Subscriber adjacency_sub_;
    ros::Publisher pub_cmd_vel_;

    double desired_heading;
    double desired_heading_past;
    double robot_orientation_past;
    double sum_heading_error;

    double max_w;

    int percent_loss_;
    std::vector<std::pair<double, double>> robot_positions_past_;
    std::vector<std::pair<double, double>> robot_velocity_past_;

    int check;
    int last_not_loss;

public:
    ConsensusProtocol(int argc, char** argv) : nh_() {
        nh_.getParam("num_robots", num_robots_);
        nh_.getParam("num_followers", num_followers_);
        num_leaders_ = num_robots_ - num_followers_;

        adjacency_row_i_.resize(num_robots_, 0);
        adjacency_sub_ = nh_.subscribe("adjacency_matrix", 10, &ConsensusProtocol::adjacencyCallback, this);

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

        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("robot_" + id + "/cmd_vel", 10);

        Kp_ = 1.0;
        Kv_ = 0.0;//0.1;
        Khp_ = 5.0;
        Khv_ = 0.0;//10.0;//1.0;
        Khi_ = 0.0;
        delta_ = 0.0;

        Kalpha_ = 5.0;
        Kbeta_ = 2.5;

        desired_heading = 0.0;
        desired_heading_past = 0.0;
        robot_orientation_past = 0.0;
        sum_heading_error = 0.0;
        max_w = 13.3;

        robot_positions_.resize(num_robots_, std::make_pair(0.0, 0.0));
        robot_orientations_.resize(num_robots_, 0.0);
        robot_velocity_.resize(num_robots_, std::make_pair(0.0, 0.0));
        odom_sub_list_.resize(num_robots_);

        robot_positions_past_.resize(num_robots_, std::make_pair(0.0, 0.0));
        robot_velocity_past_.resize(num_robots_, std::make_pair(0.0, 0.0));

        nh_.getParam("data_loss", percent_loss_);
        check = 0;
        last_not_loss = 1;
        // Test: subscribe to odom msg from all agents
        for (int i = 0; i< num_robots_;i++){
            std::string topic_name = "/robot_" + std::to_string(i) + "/odom";
            odom_sub_list_[i] = nh_.subscribe<nav_msgs::Odometry>(topic_name, 10, boost::bind(&ConsensusProtocol::odomCallback, this, _1, i));
        }
    }

    void adjacencyCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        int start_index = robot_id_ * num_robots_;
        for (int i = 0; i < num_robots_; ++i) {
            adjacency_row_i_[i] = msg->data[start_index + i];
        }
/*
        for (int i = 0; i < num_robots_; ++i) {
            if (adjacency_row_i_[i] != 0 || i == robot_id_) {
                std::string topic_name = "/robot_" + std::to_string(i) + "/odom";
                odom_sub_list_[i] = nh_.subscribe<nav_msgs::Odometry>(topic_name, 10, boost::bind(&ConsensusProtocol::odomCallback, this, _1, i));
            }
            //else if (odom_sub_list_[i]) {
            //    odom_sub_list_[i].shutdown();
            //}
        }
*/
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int robot_index) {
        double x_position = msg->pose.pose.position.x;
        double y_position = msg->pose.pose.position.y;
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double v_body = msg->twist.twist.linear.x;
        double x_dot = v_body * cos(yaw);
        double y_dot = v_body * sin(yaw);

        robot_positions_[robot_index] = std::make_pair(x_position, y_position);
        robot_orientations_[robot_index] = yaw;
        robot_velocity_[robot_index] = std::make_pair(x_dot, y_dot);

        if (check == 0){
            robot_positions_past_ = robot_positions_;
            robot_velocity_past_ = robot_velocity_;
            check = 1;
        }
    }

/*
    void pubControlLawPose(){
        std::pair<double, double> delta_position(0.0, 0.0);
        std::pair<double, double> delta_velocity(0.0, 0.0);

        double avg_theta = 0.0;
        int avg_denominator = 0;

        for (int i = 0; i < num_robots_; i++){
            if (adjacency_row_i_[i] != 0){
                int not_loss = simulateDataLoss(percent_loss_);
                delta_position.first += not_loss*adjacency_row_i_[i] * (robot_positions_[i].first - robot_positions_[robot_id_].first);
                delta_position.second += not_loss*adjacency_row_i_[i] * (robot_positions_[i].second - robot_positions_[robot_id_].second);

                avg_theta += not_loss*adjacency_row_i_[i] * robot_orientations_[i];
                avg_denominator += not_loss*adjacency_row_i_[i];
            }
        }

        avg_theta /= avg_denominator;

        double speed = Kp_ * (std::sqrt(delta_position.first * delta_position.first + delta_position.second * delta_position.second) - delta_);

        desired_heading = atan2(delta_position.second, delta_position.first);
        double angular_speed = Kalpha_ * wrap_to_pi(desired_heading - robot_orientations_[robot_id_])\
                                + 
    }*/

    void pubControlLaw() {
        std::pair<double, double> delta_position(0.0, 0.0);
        std::pair<double, double> delta_velocity(0.0, 0.0);

        for (int i = 0; i < num_robots_; i++) {
            /*if (adjacency_row_i_[i] != 0) {
                delta_position.first += adjacency_row_i_[i] * (robot_positions_[i].first - robot_positions_[robot_id_].first);
                delta_position.second += adjacency_row_i_[i] * (robot_positions_[i].second - robot_positions_[robot_id_].second);
            
                delta_velocity.first += adjacency_row_i_[i] * (robot_velocity_[i].first - robot_velocity_[robot_id_].first);
                delta_velocity.second += adjacency_row_i_[i] * (robot_velocity_[i].second - robot_velocity_[robot_id_].second);
            }*/
            // Sim Data Loss
            if (adjacency_row_i_[i] != 0) {
                int not_loss = simulateDataLoss(percent_loss_);
                delta_position.first += not_loss*adjacency_row_i_[i] * (robot_positions_[i].first - robot_positions_[robot_id_].first);
                delta_position.second += not_loss*adjacency_row_i_[i] * (robot_positions_[i].second - robot_positions_[robot_id_].second);
            
                delta_velocity.first += not_loss*adjacency_row_i_[i] * (robot_velocity_[i].first - robot_velocity_[robot_id_].first);
                delta_velocity.second += not_loss*adjacency_row_i_[i] * (robot_velocity_[i].second - robot_velocity_[robot_id_].second);
            }
            // Sim Data Loss with data loss mitigation mechanism from Xiang Gong's paper
            /*if (adjacency_row_i_[i] != 0) {
                int not_loss = simulateDataLoss(percent_loss_);

                if (not_loss == 1){
                    delta_position.first += adjacency_row_i_[i] * (robot_positions_[i].first - robot_positions_[robot_id_].first);
                    delta_position.second += adjacency_row_i_[i] * (robot_positions_[i].second - robot_positions_[robot_id_].second);
            
                    delta_velocity.first += adjacency_row_i_[i] * (robot_velocity_[i].first - robot_velocity_[robot_id_].first);
                    delta_velocity.second += adjacency_row_i_[i] * (robot_velocity_[i].second - robot_velocity_[robot_id_].second);
                }
                else {
                    if (last_not_loss == 1){
                        delta_position.first += adjacency_row_i_[i] * (robot_positions_past_[i].first - robot_positions_[robot_id_].first);
                        delta_position.second += adjacency_row_i_[i] * (robot_positions_past_[i].second - robot_positions_[robot_id_].second);
                
                        delta_velocity.first += adjacency_row_i_[i] * (robot_velocity_past_[i].first - robot_velocity_[robot_id_].first);
                        delta_velocity.second += adjacency_row_i_[i] * (robot_velocity_past_[i].second - robot_velocity_[robot_id_].second);
                    }
                    else{
                        delta_position.first += 0;
                        delta_position.second += 0;

                        delta_velocity.first += 0;
                        delta_velocity.second += 0;
                    }
                }
                robot_positions_past_[i].first = not_loss * robot_positions_[i].first;
                robot_positions_past_[i].second = not_loss * robot_positions_[i].second;
                robot_velocity_past_[i].first = not_loss * robot_velocity_[i].first;
                robot_velocity_past_[i].second = not_loss * robot_velocity_[i].second;
                last_not_loss = not_loss;
            }*/
        }
        //robot_positions_past_ = robot_positions_;
        //robot_velocity_past_ = robot_velocity_;

        double speed = Kp_ * (std::sqrt(delta_position.first * delta_position.first + delta_position.second * delta_position.second) - delta_)\
                        + Kv_ * (std::sqrt(delta_velocity.first * delta_velocity.first + delta_velocity.second * delta_velocity.second));
        
        desired_heading = atan2(delta_position.second, delta_position.first);
        sum_heading_error += wrap_to_pi(desired_heading - robot_orientations_[robot_id_]);
        if (sum_heading_error > 0.0 && sum_heading_error > max_w){
            sum_heading_error = max_w;
        }
        else if (sum_heading_error < 0.0 && sum_heading_error < -max_w){
            sum_heading_error = -max_w;
        }
        double angular_speed = Khp_ * wrap_to_pi(desired_heading - robot_orientations_[robot_id_])\
                                + Khv_ * (wrap_to_pi(desired_heading-desired_heading_past) - wrap_to_pi(robot_orientations_[robot_id_]-robot_orientation_past))\
                                + Khi_ * sum_heading_error;

        geometry_msgs::Twist twist_msg_;
        twist_msg_.linear.x = speed;
        twist_msg_.angular.z = angular_speed;
        pub_cmd_vel_.publish(twist_msg_);

        desired_heading_past = desired_heading;
        robot_orientation_past = robot_orientations_[robot_id_];
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
    ros::init(argc, argv, "simple_flock");
    ros::NodeHandle nh("~");
    ConsensusProtocol consensus_protocol(argc, argv);
    ros::Rate rate(100);
    while (ros::ok()) {
        consensus_protocol.pubControlLaw();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
