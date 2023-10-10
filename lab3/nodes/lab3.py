#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
import time


class Controller(object):
    def __init__(self):
        # publish motor commands
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )

        # rate
        self.rate = rospy.Rate(10)

        # colour index
        self.index = -1

        # define robot velocities
        self.V = 0.15 #m/s
        self.U = 0.37

    def camera_callback(self, msg):
        """Callback for line index."""
        # access the value using msg.data
        self.index = msg.data

    def bang_bang(self):
        """Implement bang bang control"""
        
        while self.index == -1:
            time.sleep(0.1)

        desired = 320
        while True:
            actual = self.index
            error = desired - actual
            if error < 0:
                correction = -self.U
            elif error > 0:
                correction = self.U
            else:
                correction = 0
            
            # publish the twist message
            twist = Twist()
            twist.linear.x = self.V
            twist.angular.z = correction
            self.cmd_pub.publish(twist)
            # rospy.loginfo(twist)
            self.rate.sleep()

    def P(self):
        while self.index == -1:
            time.sleep(0.1)
        k_p = 0.001
        desired = 320
        while True:
            actual = self.index
            error = desired - actual
            # publish the twist message
            twist = Twist()
            twist.linear.x = self.V
            twist.angular.z = error * k_p
            self.cmd_pub.publish(twist)
            # rospy.loginfo(twist)
            self.rate.sleep()

    def PI(self):
        
        while self.index == -1:
            time.sleep(0.1)
        
        k_p = 0.0025
        k_i = 0.0001

        desired = 320
        integral = 0.0

        while True:
            actual = self.index
            error = desired - actual
            integral += error
            # publish the twist message
            twist = Twist()
            twist.linear.x = self.V
            twist.angular.z = k_p * error + k_i * integral
            self.cmd_pub.publish(twist)
            # rospy.loginfo(twist)
            self.rate.sleep()

    def PID(self):
        
        while self.index == -1:
            time.sleep(0.1)
        # k_u = 0.1
        # t_u = 0.65*2
        # k_p = k_u * 0.6
        # k_i = 1.2*k_u/t_u
        # k_d = 0.075*k_u*t_u

        # k_p = 0.006
        # k_i = 0.000
        # k_d = 0.0001
        
        k_p = 0.0045
        k_i = 0.0
        k_d = 0.002

        desired = 320
        integral = 0.0
        derivative = 0.0
        lasterror = 0.0

        out_of_frame = False
        last_actual = 320

        while True:
            actual = self.index

            if out_of_frame:
                if actual < 40 or actual > 600:
                    actual = last_actual
                else:
                    # line is back in frame
                    out_of_frame = False
            else:
                if actual > 600:
                    actual = 600
                    last_actual = actual
                    out_of_frame = True
                elif actual < 40:
                    actual = 40
                    last_actual = actual
                    out_of_frame = True
                else:
                    out_of_frame = False

            error = desired - actual
            integral += error
            derivative = error - lasterror
            # publish the twist message
            twist = Twist()
            twist.linear.x = self.V
            twist.angular.z = k_p * error + k_i * integral + k_d * derivative
            self.cmd_pub.publish(twist)
            # rospy.loginfo(twist)
            lasterror = error
            self.rate.sleep()

    def follow_the_line(self, mode):
        """
        TODO: complete the function to follow the line
        """

        if mode == "bang_bang":
            self.bang_bang()
        elif mode == "P":
            self.P()
        elif mode == "PI":
            self.PI()
        elif mode == "PID":
            self.PID()


if __name__ == "__main__":
    rospy.init_node("lab3")
    controller = Controller()
    controller.follow_the_line(mode="PID")
