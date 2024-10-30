#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtle_package/MoveToTarget.h>
#include <boost/bind.hpp>

turtlesim::Pose target_pose;
bool pose_received = false;

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    target_pose = *msg;
    pose_received = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_chase");

    ros::NodeHandle nh;

    // 生成新的乌龟Group4_2
    ros::ServiceClient client_spawn = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn;
    spawn.request.x=0;
    spawn.request.y=0;
    spawn.request.theta=0;
    spawn.request.name="Group4_2";
    client_spawn.waitForExistence();

    // 6.处理响应
    if(client_spawn.call(spawn)){
        ROS_INFO("乌龟生成成功，新乌龟叫：%s",spawn.response.name.c_str());
    }
    else{
        ROS_INFO("请求失败！");
    }

    // 订阅目标乌龟Group4的位置
    ros::Subscriber pose_sub = nh.subscribe<turtlesim::Pose>("/Group4/pose", 10, poseCallback);

    // 等待接收到目标乌龟的位置
    while (ros::ok() && !pose_received) {
        ros::spinOnce();
    }

    // 控制Group4_2乌龟不停地追逐目标乌龟
    ros::ServiceClient client = nh.serviceClient<turtle_package::MoveToTarget>("move_to_target");
    turtle_package::MoveToTarget srv;
    srv.request.turtle_name = "Group4_2";

    ros::Rate rate(1000);  // 设置循环频率为10Hz
    while (ros::ok()) {
        srv.request.x = target_pose.x;
        srv.request.y = target_pose.y;
        srv.request.is_straight = false;
        srv.request.Kp_z = 20.0;
        srv.request.Kp_x = 8.0;
        if (client.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("Turtle Group4_2 moved to target successfully.");
            } else {
                ROS_WARN("Failed to move turtle Group4_2 to target.");
            }
        } else {
            ROS_ERROR("Failed to call service move_to_target");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}