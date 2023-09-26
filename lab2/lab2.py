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
    rospy.init_node("lab02")
    goal = (0.2, 0.2, 90)
    # speed is 0.2
    v = 0.1
    theta = math.atan(goal[1]/goal[0])
    x_speed = v * math.sin(theta)
    # y_speed = v * math.cos(theta)
    time = goal[0]/x_speed
    angular_speed = goal[2]/time
    twist = Twist()
    # time = 5
    # twist.angular.z = -math.pi/2 /5
    # twist.linear.x = 0.2/5
    # twist.linear.y = y_speed
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    rt = 30
    rate = rospy.Rate(rt)
    t = 0
    goal = (2, 0.5, 135*math.pi/180)
    while not rospy.is_shutdown():
        twist.linear.x = 0.2
        cmd_pub.publish(twist)
        rate.sleep()
        t += 1
        if 0.2*t/rt >= 1.75:
            break
    twist.linear.x = 0
    time = 0
    while not rospy.is_shutdown():
        twist.linear.x = 0.1
        twist.angular.z = -0.2839
        cmd_pub.publish(twist)
        rate.sleep()
        time += 1
        if 0.2839 * time/rt >= goal[2]:
            break
        

if __name__ == "__main__":
    main()



# #!/usr/bin/env python
# import rospy
# import math
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String
# from time import sleep
# from nav_msgs.msg import Odometry

# cur_robot_pose = (0.0, 0.0, 0.0)


# def sanitize_angle(theta):
#     while theta < 0:
#         theta += 2 * math.pi
#     while theta >= 2 * math.pi:
#         theta -= 2 * math.pi
#     return theta


# def get_yaw_from_quaternion(q):
#     siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#     yaw = math.atan2(siny_cosp, cosy_cosp)
#     return yaw


# def callback(odom_data):
#     """TODO: complete the call back function for subscriber"""
#     global cur_robot_pose

#     point = odom_data.pose.pose.position
#     quart = odom_data.pose.pose.orientation
#     theta = get_yaw_from_quaternion(quart)
#     cur_pose = (point.x, point.y, theta)

#     cur_robot_pose = cur_pose

#     rospy.loginfo(cur_pose)


# def point_rotate(cmd_pub, target_angle):
#     """ target angle in rad """
#     global cur_robot_pose
#     rospy.wait_for_message("/odom", Odometry, timeout=None)
#     rate = rospy.Rate(100)
#     twist = Twist()
#     while not rospy.is_shutdown():
#         # r = cur_robot_pose[2] if cur_robot_pose[2] > 0 else cur_robot_pose[2] + 2*math.pi
#         if abs(sanitize_angle(cur_robot_pose[2]) - target_angle) < 0.05:
#             print("DEBUG:", sanitize_angle(cur_robot_pose[2]), target_angle)
#             print("DEBUG:", sanitize_angle(cur_robot_pose[2]) - target_angle)
#             break
#         print("DEBUG:", sanitize_angle(cur_robot_pose[2]), target_angle)
        
#         twist.angular.z = -0.3
#         cmd_pub.publish(twist)
#         rospy.loginfo(twist)
#         rate.sleep()
#     twist.angular.z = 0
#     cmd_pub.publish(twist)


# def straight_drive(cmd_pub, target_dist):
#     global cur_robot_pose
#     rospy.wait_for_message("/odom", Odometry, timeout=None)
#     start_pos = cur_robot_pose

#     rate = rospy.Rate(100)
#     twist = Twist()
#     while not rospy.is_shutdown() and math.sqrt((start_pos[0] - cur_robot_pose[0])**2+(start_pos[1] - cur_robot_pose[1])**2) < target_dist:

#         print("DEBUG:", math.sqrt((start_pos[0] - cur_robot_pose[0])**2 + (start_pos[1] - cur_robot_pose[1])**2), target_dist)
#         twist.linear.x = 0.26
#         cmd_pub.publish(twist)
#         rospy.loginfo(twist)
#         rate.sleep()
#     twist.linear.x = 0
#     cmd_pub.publish(twist)
#     print("DEBUG: fssdf", math.sqrt((start_pos[0] - cur_robot_pose[0])**2 + (start_pos[1] - cur_robot_pose[1])**2), target_dist)

# def main():
#     global cur_robot_pose
#     rospy.init_node("lab02")
#     odom_subscriber = rospy.Subscriber('odom', Odometry, callback, queue_size=1)

#     """TODO: publish commands to make robot move to desired pose"""

#     cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
#     # start_angle = cur_robot_pose[2]
#     b = 1
#     goal = (b, 0, math.pi/2)
#     start_angle = cur_robot_pose[2]
#     a = start_angle - 0
#     rospy.wait_for_message("/odom", Odometry, timeout=None)
#     # point_rotate(cmd_pub, sanitize_angle(a))
#     straight_drive(cmd_pub, math.sqrt(goal[0]**2 + goal[1]**2))
#     point_rotate(cmd_pub, sanitize_angle((start_angle - goal[2])))
#     goal = (b, b, math.pi)
#     # a = start_angle - math.pi/4
#     # point_rotate(cmd_pub, sanitize_angle(a))
#     straight_drive(cmd_pub, math.sqrt(goal[0]**2 + goal[1]**2))
#     point_rotate(cmd_pub, sanitize_angle((start_angle - goal[2])))
#     goal = (0, b, 270*math.pi/180)
#     # a = start_angle - math.pi/2
#     # point_rotate(cmd_pub, sanitize_angle(a))
#     straight_drive(cmd_pub, math.sqrt(goal[0]**2 + goal[1]**2))
#     point_rotate(cmd_pub, sanitize_angle((start_angle - goal[2])))

#     # straight_drive(cmd_pub, 0.2)
#     # rate = rospy.Rate(10)
#     # counter = 0
#     # while not rospy.is_shutdown():
#     #     counter += 1
#     #     if counter >= 70:
#     #         break
#     #     twist = Twist()
#     #     twist.linear.x = 0.5
#     #     cmd_pub.publish(twist)
#     #     rospy.loginfo(twist)
#     #     rate.sleep()
#     # twist.linear.x = 0
#     # cmd_pub.publish(twist)
#     # while not rospy.is_shutdown():
#     #     counter += 1
#     #     if counter >= 114:
#     #         break
#     #     twist = Twist()
#     #     twist.angular.z = -1
#     #     cmd_pub.publish(twist)
#     #     rospy.loginfo(twist)
#     #     rate.sleep()
#     # twist.angular.z = 0
#     # cmd_pub.publish(twist)

# if __name__ == "__main__":
#     main()
