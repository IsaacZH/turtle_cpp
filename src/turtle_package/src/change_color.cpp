#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <cstdlib> // 用于生成随机数
#include <ctime>   // 用于生成随机数种子
#include <unistd.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "background");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/clear");

    srand(time(0)); // 初始化随机数种子

    while (ros::ok())
    {
        nh.setParam("/turtle1/background_r", rand() % 256);
        nh.setParam("/turtle1/background_b", rand() % 256);
        nh.setParam("/turtle1/background_g", rand() % 256);

        // 等待服务可用
        client.waitForExistence();

        std_srvs::Empty srv;
        if (client.call(srv))
        {
            ROS_INFO("Cleared!");
        }
        else
        {
            ROS_ERROR("Failed to call /clear service");
        }

        usleep(1000000); // 休眠1秒
    }

    return 0;
}
