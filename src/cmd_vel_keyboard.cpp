#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ncurses.h>
#include <unordered_map>
#include <unordered_set>

int main(int argc, char **argv) {
    if (argc < 2) {
        ROS_ERROR("Please provide the robot ID as a command-line argument.");
        return 1;
    }
    std::string robot_id = argv[1];

    ros::init(argc, argv, "cmd_vel_keyboard");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/robot_" + robot_id + "/cmd_vel", 10);
    ros::Rate rate(100);

    geometry_msgs::Twist twist_msg;

    std::unordered_map<int, std::pair<double, double>> key_mapping = {
        {KEY_UP, {1.0, 0.0}},
        {KEY_DOWN, {-1.0, 0.0}},
        {KEY_LEFT, {0.0, 1.0}},
        {KEY_RIGHT, {0.0, -1.0}}
    };

    double linear_vel = 0.0;
    double angular_vel = 0.0;
    std::unordered_set<int> pressed_keys;

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    timeout(0); // Non-blocking input

    while (ros::ok()) {
        int ch = getch();

        if (ch != ERR) {
            if (key_mapping.count(ch)) {
                pressed_keys.insert(ch);
            }
        } else {
            pressed_keys.clear();
        }

        linear_vel = 0.0;
        angular_vel = 0.0;
        for (int key : pressed_keys) {
            linear_vel += key_mapping[key].first;
            angular_vel += key_mapping[key].second;
        }

        twist_msg.linear.x = linear_vel;
        twist_msg.angular.z = angular_vel;
        pub.publish(twist_msg);

        rate.sleep();
    }

    endwin();
    return 0;
}