import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from threading import Lock
from concurrent.futures import ThreadPoolExecutor
from turtle_cpp.srv import MoveToTarget, MoveToTargetResponse

class TargetInfo:
    def __init__(self, x, y, Kp_z, Kp_x, is_straight):
        self.x = x
        self.y = y
        self.Kp_z = Kp_z
        self.Kp_x = Kp_x
        self.is_straight = is_straight

turtle_poses = {}
turtle_publishers = {}
turtle_subscribers = {}
pose_mutex = Lock()

def pose_callback(pose_message, turtle_name):
    with pose_mutex:
        turtle_poses[turtle_name] = pose_message

def move_to_target(turtle_name, target, rate):
    vel_msg = Twist()
    Kp_z = target.Kp_z
    Kp_x = target.Kp_x
    is_straight = target.is_straight
    last_print_time = rospy.Time.now()

    while not rospy.is_shutdown():
        with pose_mutex:
            turtle_pose = turtle_poses.get(turtle_name)
            if turtle_pose is None:
                continue

        distance = math.sqrt((target.x - turtle_pose.x) ** 2 + (target.y - turtle_pose.y) ** 2)
        if distance < 0.01:
            break

        angle_to_goal = math.atan2(target.y - turtle_pose.y, target.x - turtle_pose.x)
        if is_straight:
            angle_to_goal = round(angle_to_goal / (math.pi / 2)) * (math.pi / 2)
        angle_error = angle_to_goal - turtle_pose.theta

        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        if abs(angle_error) > 0.005:
            vel_msg.angular.z = Kp_z * angle_error
        else:
            vel_msg.angular.z = 0
            vel_msg.linear.x = Kp_x * distance

        if (rospy.Time.now() - last_print_time).to_sec() >= 0.5:
            last_print_time = rospy.Time.now()

        turtle_publishers[turtle_name].publish(vel_msg)
        rate.sleep() 

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    turtle_publishers[turtle_name].publish(vel_msg)

def handle_move_to_target(req):
    turtle_name = req.turtle_name
    target = TargetInfo(req.x, req.y, req.Kp_z, req.Kp_x, req.is_straight)
    rate = rospy.Rate(1000)  # 修改为10Hz

    if turtle_name not in turtle_publishers:
        pose_subscriber = rospy.Subscriber(f"/{turtle_name}/pose", Pose, pose_callback, turtle_name)
        velocity_publisher = rospy.Publisher(f"/{turtle_name}/cmd_vel", Twist, queue_size=10)
        turtle_subscribers[turtle_name] = pose_subscriber
        turtle_publishers[turtle_name] = velocity_publisher

    with ThreadPoolExecutor() as executor:
        executor.submit(move_to_target, turtle_name, target, rate)

    return MoveToTargetResponse(success=True)

if __name__ == "__main__":
    rospy.init_node('turtle_move_server')
    service = rospy.Service('move_to_target', MoveToTarget, handle_move_to_target)
    rospy.loginfo("Ready to move turtles to target.")
    rospy.spin()
