#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class DifferentialDriveKinematics {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::Transform transform_;
    nav_msgs::Odometry odom_;
    double x_, y_, theta_;
    double limit_v_, limit_w_;
    ros::Time last_time_;
    ros::Rate rate_;

    double clamp(double val, double min_val, double max_val) {
        return std::max(min_val, std::min(val, max_val));
    }

public:
    DifferentialDriveKinematics(int argc, char** argv) : nh_("~"), rate_(100) {
        std::string id;
        if (argc >= 2){
            id = argv[1];
        }
        else {
            ROS_ERROR("Robot ID argument not provided. Exiting...");
            ros::shutdown();
            return;   
        }

        cmd_vel_sub_ = nh_.subscribe("/robot_" + id + "/cmd_vel", 10, &DifferentialDriveKinematics::cmdVelCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/robot_" + id + "/odom", 10);

        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_link_" + id;

        nh_.param("init_x_" + id, x_, 0.0);
        nh_.param("init_y_" + id, y_, 0.0);
        nh_.param("init_theta_" + id, theta_, 0.0);
        nh_.param("limit_v", limit_v_, 2.0);
        nh_.param("limit_w", limit_w_, 13.3);

        last_time_ = ros::Time::now();

        // Set the initial values for the odom_ message
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

        odom_.pose.pose.position.x = x_;
        odom_.pose.pose.position.y = y_;
        odom_.pose.pose.orientation = odom_quat;   

        // Broadcasting the initial TF
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x_, y_, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta_);
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", odom_.child_frame_id));    
    }

    void cmdVelCallback(const geometry_msgs::Twist& twist_msg) {
        ros::Time current_time = ros::Time::now();
        double delta_time = (current_time - last_time_).toSec();

        double linear_velocity = clamp(twist_msg.linear.x, -limit_v_, limit_v_);
        double angular_velocity = clamp(twist_msg.angular.z, -limit_w_, limit_w_);

        double delta_theta = angular_velocity * delta_time;
        double delta_x = linear_velocity * cos(theta_) * delta_time;
        double delta_y = linear_velocity * sin(theta_) * delta_time;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

        odom_.header.stamp = current_time;
        odom_.pose.pose.position.x = x_;
        odom_.pose.pose.position.y = y_;
        odom_.pose.pose.orientation = odom_quat;

        odom_.twist.twist.linear.x = linear_velocity;
        odom_.twist.twist.angular.z = angular_velocity;

        // Broadcasting TF after updating the pose
        //tf::Transform transform;
        transform_.setOrigin(tf::Vector3(x_, y_, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta_);
        transform_.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "odom", odom_.child_frame_id));

        last_time_ = current_time;
    }

    void spin() {
        while (ros::ok()) {
            odom_pub_.publish(odom_);
            
            rate_.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dd_kinematics");
    DifferentialDriveKinematics node(argc, argv);
    node.spin();
    return 0;
}
