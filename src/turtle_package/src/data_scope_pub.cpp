#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>
#include <std_msgs/Float64.h> 

// 全局变量存储海龟的位置和朝向
turtlesim::Pose group4_pose;
turtlesim::Pose group4_2_pose;

// 回调函数更新Group4的位置
void group4PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    group4_pose = *msg;
}

// 回调函数更新Group4_2的位置和朝向
void group4_2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    group4_2_pose = *msg;
}

// 计算两只海龟之间的直线距离
double calculateDistance(const turtlesim::Pose& pose1, const turtlesim::Pose& pose2) {
    return sqrt(pow(pose1.x - pose2.x, 2) + pow(pose1.y - pose2.y, 2));
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "data_scope_pub");
    ros::NodeHandle nh;

    // 订阅两只海龟的位置话题
    ros::Subscriber group4_sub = nh.subscribe("/Group4/pose", 10, group4PoseCallback);
    ros::Subscriber group4_2_sub = nh.subscribe("/Group4_2/pose", 10, group4_2PoseCallback);

    // 创建发布者发布距离和朝向角度
    ros::Publisher distance_pub = nh.advertise<std_msgs::Float64>("/turtle_distance", 10);
    ros::Publisher orientation_pub = nh.advertise<std_msgs::Float64>("/turtle_2_orientation", 10);

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok()) {
        ros::spinOnce();

        // 计算距离并发布
        std_msgs::Float64 distance_msg;
        distance_msg.data = calculateDistance(group4_pose, group4_2_pose);
        distance_pub.publish(distance_msg);

        // 发布Group4_2的朝向角度
        std_msgs::Float64 orientation_msg;
        orientation_msg.data = group4_2_pose.theta;
        orientation_pub.publish(orientation_msg);

        // 打印距离和角度
        ROS_INFO("Group4_Pose - X: %f, Y: %f, Theta: %f    Distance: %f, Group4_2 theta: %f", group4_pose.x, group4_pose.y, group4_pose.theta, distance_msg.data, group4_2_pose.theta);
        loop_rate.sleep();
    }

    return 0;
}