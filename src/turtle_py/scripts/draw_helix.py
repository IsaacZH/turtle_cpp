#!/usr/bin/env python
import rospy
from turtle_py.srv import MoveToTarget
import sys

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def hilbert(n, points, x0, y0):
    if n <= 0:
        points.append(Point(x0, y0))
    else:
        prev_points = []
        hilbert(n - 1, prev_points, 0.0, 0.0)
        for p in prev_points:
            points.append(Point(0.5 * (-0.5 + p.y), 0.5 * (-0.5 + p.x)))
        for p in prev_points:
            points.append(Point(0.5 * (-0.5 + p.x), 0.5 * (0.5 + p.y)))
        for p in prev_points:
            points.append(Point(0.5 * (0.5 + p.x), 0.5 * (0.5 + p.y)))
        for p in prev_points:
            points.append(Point(0.5 * (0.5 - p.y), 0.5 * (-0.5 - p.x)))

def main():
    rospy.init_node('draw_hilbert_curve')
    rospy.wait_for_service('move_to_target')  
    move_to_target = rospy.ServiceProxy('move_to_target', MoveToTarget)
    order = int(sys.argv[1])
    points = []
    hilbert(order, points, 0, 0)

    for point in points:
        point.y *= -1
        point.x *= -1

    w, h = 11.088889, 11.088889
    for point in points:
        point.x = (point.x + 0.5) * w
        point.y = (point.y + 0.5) * h

    for i, point in enumerate(points):
        while not rospy.is_shutdown():
            try:
                move_to_target('Group4', point.x, point.y, 15.0, 6.0, i != 0)  
                break
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed, retrying...")
                rospy.sleep(1.0)

if __name__ == '__main__':
    main()