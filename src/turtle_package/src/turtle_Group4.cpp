// 1.头文件
#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Kill.h"
 

//实时位置输出接受函数
void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    // ROS_INFO("Group4_Pose - X: %f, Y: %f, Theta: %f", msg->x, msg->y, msg->theta);
}

int main(int argc, char *argv[])
{
    //生成Group
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"service_call");
    ros::NodeHandle nh;

    ros::ServiceClient client= nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn;
    spawn.request.x=10;
    spawn.request.y=8;
    spawn.request.theta=0;
    spawn.request.name="Group4";
    client.waitForExistence();

    // 6.处理响应
    if(client.call(spawn)){
        ROS_INFO("乌龟生成成功，新乌龟叫：%s",spawn.response.name.c_str());
    }
    else{
        ROS_INFO("请求失败！");
    }
    //订阅乌龟Group4的位置
    ros::Subscriber PoseSub = nh.subscribe("/Group4/pose", 1000, poseCallback);

    ros::spin();
    
    return 0;
}
