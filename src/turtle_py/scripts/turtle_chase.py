#!/usr/bin/env python
import rospy
from turtlesim.srv import Spawn, SpawnRequest
from turtlesim.msg import Pose
from turtle_py.srv import MoveToTarget, MoveToTargetRequest

target_pose = Pose()
pose_received = False

def pose_callback(msg):
    global target_pose, pose_received
    target_pose = msg
    pose_received = True

def main():
    global pose_received
    rospy.init_node('turtle_chase')
    try:
        client = rospy.ServiceProxy('/spawn', Spawn)
        client.wait_for_service() # 等待服务启动
        request = SpawnRequest()
        request.x = 0.0
        request.y = 0.0
        request.theta = 0.0
        request.name = 'Group4_2'
        response = client.call(request) 
        rospy.loginfo("乌龟生成成功，新乌龟叫：%s", response.name)
    except rospy.ServiceException as e:
        rospy.loginfo("请求失败: %s" % e)

    rospy.Subscriber('/Group4/pose', Pose, pose_callback)

    while not pose_received and not rospy.is_shutdown():
        rospy.sleep(0.1)

    move_to_target = rospy.ServiceProxy('move_to_target', MoveToTarget)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            move_to_target('Group4_2', target_pose.x, target_pose.y, 20.0, 8.0, False)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        rate.sleep()

if __name__ == '__main__':
    main()