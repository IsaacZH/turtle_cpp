#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "turtlesim/Pose.h"
#include <vector>
#include <cmath>
#include <turtle_package/MoveToTarget.h>

struct Point {
    double x, y;
};

// 生成希尔伯特曲线的递归函数
void hilbert(int n, std::vector<Point>& points, double x0, double y0) {
    if (n <= 0) {
        points.push_back(Point{x0, y0}); // 添加坐标点
    } else {
        // 存储上一层的坐标
        std::vector<Point> prev_points;
        hilbert(n - 1, prev_points, 0.0, 0.0); // 递归获取前一层的点

        // 进行坐标转换
        for (const auto& p : prev_points) {
            points.push_back(Point{0.5 * (-0.5 + p.y), 0.5 * (-0.5 + p.x)});
        }
        for (const auto& p : prev_points) {
            points.push_back(Point{0.5 * (-0.5 + p.x), 0.5 * (0.5 + p.y)});
        }
        for (const auto& p : prev_points) {
            points.push_back(Point{0.5 * (0.5 + p.x), 0.5 * (0.5 + p.y)});
        }
        for (const auto& p : prev_points) {
            points.push_back(Point{0.5 * (0.5 - p.y), 0.5 * (-0.5 - p.x)});
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "draw_hilbert_curve");
    ros::NodeHandle nh;
    // 创建服务客户端
    ros::ServiceClient client = nh.serviceClient<turtle_package::MoveToTarget>("move_to_target");
    turtle_package::MoveToTarget srv;

    int order = 2;  // 希尔伯特曲线的阶数
    std::vector<Point> points;
    hilbert(order,points,0,0);   

    for (auto& point : points) {
        point.y = point.y * -1;
        point.x = point.x * -1;
    }
    // 将生成的点缩放到乌龟模拟器的范围内
    double w = 11.088889;
    double h = 11.088889;
    for (auto& point : points) {
        point.x = (point.x + 0.5) * w;
        point.y = (point.y + 0.5) * h;
    }

    // 画希尔伯特曲线
    srv.request.turtle_name = "Group4";
    
    srv.request.Kp_z = 15.0;
    srv.request.Kp_x = 6.0;
    for (size_t i = 0; i < points.size(); ++i) {
        // 设置服务请求参数
        srv.request.x = points[i].x;
        srv.request.y = points[i].y;
        srv.request.is_straight = (i != 0);// 从第二个点开始直线运动
    
        // 调用服务并等待响应成功
        while (true) {
            if (client.call(srv)) {
                if (srv.response.success) {
                    break;  // 成功响应，跳出循环发送下一个点
                } else {
                    ROS_WARN("Service call failed, retrying...");
                }
            } else {
                ROS_ERROR("Failed to call service");
                ros::Duration(1.0).sleep();  // 等待1秒后重试
            }
        }
    }
    
    return 0;
}