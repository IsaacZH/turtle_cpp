#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>
#include <unordered_map>
#include <mutex>
#include <future>
#include <thread>
#include <turtle_cpp/MoveToTarget.h>  // 包含服务消息类型

struct target_info {
    double x, y;
    double Kp_z, Kp_x;
    bool is_straight;
};

std::unordered_map<std::string, turtlesim::Pose> turtle_poses; // 保存乌龟的位置信息
std::unordered_map<std::string, ros::Publisher> turtle_publishers; // 保存乌龟的速度发布器
std::unordered_map<std::string, ros::Subscriber> turtle_subscribers; // 保存乌龟的位置订阅器
std::mutex pose_mutex; // 互斥锁

void poseCallback(const turtlesim::Pose::ConstPtr& pose_message, const std::string& turtle_name) {
    std::lock_guard<std::mutex> lock(pose_mutex); // 加锁
    turtle_poses[turtle_name] = *pose_message; // 更新乌龟的位置信息
}

void Move_To_Target(const std::string& turtle_name, const target_info& target, ros::Rate& rate) {
    geometry_msgs::Twist vel_msg;
    double Kp_z = target.Kp_z;
    double Kp_x = target.Kp_x;
    bool is_straight = target.is_straight;
    // Kp_z = 15;  // 比例增益
    // Kp_x = 6;  // 比例增益
    ros::Time last_print_time = ros::Time::now();

    while (ros::ok()) {
        pose_mutex.lock();
        turtlesim::Pose turtle_pose = turtle_poses[turtle_name];
        pose_mutex.unlock();

        double distance = std::sqrt(std::pow(target.x - turtle_pose.x, 2) +
                                    std::pow(target.y - turtle_pose.y, 2));
        if (distance < 0.01) {
            // ROS_INFO("Reached the target point");
            break;
        }

        double angle_to_goal = std::atan2(target.y - turtle_pose.y, target.x - turtle_pose.x);
        if (is_straight) {
            angle_to_goal = std::round(angle_to_goal / (M_PI / 2)) * (M_PI / 2);
        }
        double angle_error = angle_to_goal - turtle_pose.theta;
        // 过零点检测
        if (angle_error > M_PI) {
            angle_error -= 2 * M_PI;
        } else if (angle_error < -M_PI) {
            angle_error += 2 * M_PI;
        }

        if (std::abs(angle_error) > 0.005) {
            vel_msg.angular.z = Kp_z * angle_error;
        } else {
            vel_msg.angular.z = 0;
            vel_msg.linear.x = Kp_x * distance;
        }


        if ((ros::Time::now() - last_print_time).toSec() >= 0.5) {
            // ROS_INFO("Turtle: %s, Target velocity - linear.x: %f, angle: %f", turtle_name.c_str(), vel_msg.linear.x, angle_to_goal);
            // ROS_INFO("Theta: %f", turtle_pose.theta);
            // ROS_INFO("target.x: %f, target.y: %f,target.Kp_z: %f, target.Kp_x: %f", target.x, target.y, target.Kp_z, target.Kp_x);
            last_print_time = ros::Time::now();
        }

        turtle_publishers[turtle_name].publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }

    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    turtle_publishers[turtle_name].publish(vel_msg);
}

bool handleMoveToTarget(turtle_cpp::MoveToTarget::Request &req, turtle_cpp::MoveToTarget::Response &res) {
    std::string turtle_name = req.turtle_name;
    target_info target;
    target.x = req.x;
    target.y = req.y;
    target.Kp_z = req.Kp_z;
    target.Kp_x = req.Kp_x;
    target.is_straight = req.is_straight;
    ros::Rate rate(1000);

    if (turtle_publishers.find(turtle_name) == turtle_publishers.end()) {
        // 动态订阅乌龟的位置信息
        ros::NodeHandle nh;
        ros::Subscriber pose_subscriber = nh.subscribe<turtlesim::Pose>("/" + turtle_name + "/pose", 10, boost::bind(poseCallback, _1, turtle_name));
        ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>("/" + turtle_name + "/cmd_vel", 10);
        turtle_subscribers[turtle_name] = pose_subscriber;
        turtle_publishers[turtle_name] = velocity_publisher;
    }
    // Move_To_Target(turtle_name, target, rate, req.is_straight);

    std::async(std::launch::async, Move_To_Target, turtle_name, target, std::ref(rate));    
    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_move_server");
    ros::NodeHandle nh;

    ros::Time::init();
    // 创建一个多线程的服务服务器
    ros::AsyncSpinner spinner(4);  // 使用4个线程
    spinner.start();

    ros::ServiceServer service = nh.advertiseService("move_to_target", handleMoveToTarget);

    ROS_INFO("Ready to move turtles to target.");
    ros::waitForShutdown();

    return 0;
}