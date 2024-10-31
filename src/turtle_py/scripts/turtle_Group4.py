import rospy
from turtlesim.srv import Spawn, SpawnRequest

if __name__ == '__main__':
    rospy.init_node('service_call')
    try:
        client = rospy.ServiceProxy('/spawn', Spawn)
        client.wait_for_service() # 等待服务启动
        request = SpawnRequest()
        request.x = 10.0
        request.y = 8.0
        request.theta = 0.0
        request.name = 'Group4'
        response = client.call(request) 
        rospy.loginfo("乌龟生成成功，新乌龟叫：%s", response.name)
    except rospy.ServiceException as e:
        rospy.loginfo("请求失败: %s" % e)
