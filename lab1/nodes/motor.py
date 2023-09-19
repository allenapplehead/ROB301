#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from time import sleep


def main():
    rospy.init_node("motor_node")

    """TODO: complete motor publishing functionality here"""
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    rate = rospy.Rate(10)
    counter = 0
    while not rospy.is_shutdown():
        counter += 1
        if counter >= 70:
            break
        twist = Twist()
        twist.linear.x = 0.5
        cmd_pub.publish(twist)
        rospy.loginfo(twist)
        rate.sleep()
    twist.linear.x = 0
    cmd_pub.publish(twist)
    while not rospy.is_shutdown():
        counter += 1
        if counter >= 114:
            break
        twist = Twist()
        twist.angular.z = -1
        cmd_pub.publish(twist)
        rospy.loginfo(twist)
        rate.sleep()
    twist.angular.z = 0
    cmd_pub.publish(twist)
    
    #sleep(10)
    #twist.linear.x = 0
    #cmd_pub.publish(twist)
    #twist.angular.z = 0.1
    #cmd_pub.publish(twist)
    #sleep(6.283)
    #twist.angular.z = 0
    #cmd_pub.publish(twist)


if __name__ == "__main__":
    main()
