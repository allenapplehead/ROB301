#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from time import sleep
from nav_msgs.msg import Odometry

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def callback(odom_data):
    """TODO: complete the call back function for subscriber"""
    global cur_robot_pose

    point = odom_data.pose.pose.position
    quart = odom_data.pose.pose.orientation
    theta = get_yaw_from_quaternion(quart)
    cur_pose = (point.x, point.y, theta)

    cur_robot_pose = cur_pose

    rospy.loginfo(cur_pose)

def main():
    goal = (0.1, 0.1, 90)
    # speed is 0.2
    v = 0.2
    theta = math.atan(goal[1]/goal[0])
    x_speed = v * math.sin(theta)
    y_speed = v * math.cos(theta)
    time = goal[0]/x_speed
    angular_speed = goal[2]/time
    twist = Twist()
    twist.angular.z = -angular_speed
    twist.linear.x = x_speed
    twist.linear.y = y_speed
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    rt = 100
    rate = rospy.Rate(rt)
    t = 0
    while not rospy.is_shutdown() and t < time * rt:
        cmd_pub.publish(twist)
        rate.sleep()
        t += 1


if __name__ == "__main__":
    main()