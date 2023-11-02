#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

class RobotPlotter {
public:
    RobotPlotter() : nodeHandle("~"), img(500, 500, CV_8UC3) {
        nodeHandle.param("num_robots", numRobots, 1);
        
        robotPositions.resize(numRobots, std::make_pair(0.0, 0.0));
        robotOrientations.resize(numRobots, 0.0);

        nodeHandle.param("xlim_min", xlimMin, -5.0);
        nodeHandle.param("xlim_max", xlimMax, 5.0);
        nodeHandle.param("ylim_min", ylimMin, -5.0);
        nodeHandle.param("ylim_max", ylimMax, 5.0);

        for (int i = 0; i < numRobots; i++) {
            std::string topic = "/robot_" + std::to_string(i) + "/odom";
            subscribers.push_back(nodeHandle.subscribe<nav_msgs::Odometry>(topic, 10, boost::bind(&RobotPlotter::odometryCallback, this, _1, i)));
        }

        cv::namedWindow("Robot Positions and Orientations", cv::WINDOW_AUTOSIZE);
    }

    cv::RotatedRect create_robot_triangle(const cv::Point2f &center, double yaw, double side_length = 0.3, double front_length = 0.45) {
        double half_length = side_length / 2;
        cv::Point2f vertices[3];

        vertices[0] = cv::Point2f(center.x - half_length, center.y - half_length);
        vertices[1] = cv::Point2f(center.x - half_length, center.y + half_length);
        vertices[2] = cv::Point2f(center.x + front_length, center.y);

        cv::RotatedRect rotated_rect(center, cv::Size2f(side_length, front_length), yaw * (180.0 / CV_PI)); // Convert yaw to degrees
        return rotated_rect;
    }

    std::vector<cv::Point2f> robot_positions;
    std::vector<double> robot_orientations;

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg, int robot_index) {
        float x_position = msg->pose.pose.position.x;
        float y_position = msg->pose.pose.position.y;
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        robotPositions[robot_index] = std::make_pair(x_position, y_position);
        robotOrientations[robot_index] = yaw;
    }


    void updatePlot() {
        img.setTo(cv::Scalar(255, 255, 255));  // Clear the image
        
        for (size_t i = 0; i < robotPositions.size(); i++) {
            cv::Point center(robotPositions[i].first * 50 + img.cols/2, robotPositions[i].second * 50 + img.rows/2);  // Simple scaling by 50 for visualization
            cv::Scalar color = cv::Scalar(0, 0, 255); // Red for leader

            cv::RotatedRect triangle = create_robot_triangle(center, robotOrientations[i]);
            cv::Point2f vertices[3];
            triangle.points(vertices);

            cv::line(img, vertices[0], vertices[1], color, 2);
            cv::line(img, vertices[1], vertices[2], color, 2);
            cv::line(img, vertices[2], vertices[0], color, 2);
            
        }

        cv::imshow("Robot Positions and Orientations", img);
        cv::waitKey(30);  // Update every 30ms
    }


    void run() {
        ros::Rate rate(30);  // 30 Hz
        while (ros::ok()) {
            updatePlot();
            rate.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nodeHandle;
    int numRobots;
    double xlimMin, xlimMax, ylimMin, ylimMax;
    std::vector<std::pair<double, double>> robotPositions;
    std::vector<double> robotOrientations;
    std::vector<ros::Subscriber> subscribers;
    cv::Mat img;  // OpenCV image for plotting
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planar_simulation");
    RobotPlotter plotter;
    plotter.run();
    return 0;
}
