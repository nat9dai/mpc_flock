#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cmath>

class ConsensusProtocol {
private:
    ros::NodeHandle nh_;
    int num_robots_;
    int num_followers_;
    int num_leaders_;
    int robot_id_;
    std::vector<int> adjacency_row_i_;
    std::vector<std::pair<double, double>> robot_positions_;
    std::vector<double> robot_orientations_;
    std::vector<ros::Subscriber> odom_sub_list_;
    double Kv_;
    double Kh_;
    ros::Subscriber adjacency_sub_;
    ros::Publisher pub_cmd_vel_;

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

        Kv_ = 0.5;
        Kh_ = 5.0;

        robot_positions_.resize(num_robots_, std::make_pair(0.0, 0.0));
        robot_orientations_.resize(num_robots_, 0.0);
        odom_sub_list_.resize(num_robots_);
    }

    void adjacencyCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        int start_index = robot_id_ * num_robots_;
        for (int i = 0; i < num_robots_; ++i) {
            adjacency_row_i_[i] = msg->data[start_index + i];
        }

        for (int i = 0; i < num_robots_; ++i) {
            if (adjacency_row_i_[i] != 0 || i == robot_id_) {
                std::string topic_name = "/robot_" + std::to_string(i) + "/odom";
                // The odomCallback doesn't use the i value, so no need to bind
                odom_sub_list_[i] = nh_.subscribe<nav_msgs::Odometry>(topic_name, 10, &ConsensusProtocol::odomCallback, this);
            } else if (odom_sub_list_[i]) {
                odom_sub_list_[i].shutdown();
            }
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        int robot_index = robot_id_;
        double x_position = msg->pose.pose.position.x;
        double y_position = msg->pose.pose.position.y;
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        robot_positions_[robot_index] = std::make_pair(x_position, y_position);
        robot_orientations_[robot_index] = yaw;
    }

    void pubControlLaw() {
        std::pair<double, double> delta_position(0.0, 0.0);

        for (int i = 0; i < num_robots_; i++) {
            if (adjacency_row_i_[i] != 0) {
                delta_position.first += adjacency_row_i_[i] * (robot_positions_[i].first - robot_positions_[robot_id_].first);
                delta_position.second += adjacency_row_i_[i] * (robot_positions_[i].second - robot_positions_[robot_id_].second);
            }
        }

        double desired_heading = atan2(delta_position.second, delta_position.first);
        double speed = Kv_ * std::sqrt(delta_position.first * delta_position.first + delta_position.second * delta_position.second);
        double angular_speed = Kh_ * wrap_to_pi(desired_heading - robot_orientations_[robot_id_]);  // Assuming you have robot_orientations_ defined similarly to robot_positions_

        geometry_msgs::Twist twist_msg_;
        twist_msg_.linear.x = speed;
        twist_msg_.angular.z = angular_speed;
        pub_cmd_vel_.publish(twist_msg_);
    }

    double wrap_to_pi(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_flock");
    ros::NodeHandle nh("~");
    ConsensusProtocol consensus_protocol(argc, argv);
    ros::Rate rate(60);
    while (ros::ok()) {
        consensus_protocol.pubControlLaw();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
