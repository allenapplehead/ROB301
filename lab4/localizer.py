#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import UInt32

import matplotlib.pyplot as plt
import math
import numpy as np
import time

x_plot = []
meas = []
cov = []

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0

        self.u = 0.1  # initialize the cmd_vel input
        self.phi = 0  # initialize the measurement input

        ## lab3 stuff -----
        # publish motor commands
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )

        # colour index
        self.index = -1

        # define robot velocities
        # self.V = 0.1 #m/s
        # self.U = 0.7

        # define camera correction bounds
        self.L = 40
        self.R = 600

        self.prev_phi = 0
        ## lab4 stuff -----
        self.rate = rospy.Rate(10) # initialize rospy timestep

        # specify the 4 mail addresses
        self.addy_1 = 0.61
        self.addy_2 = 1.12
        self.addy_3 = 2.44
        self.addy_4 = 3.15
        self.addy = [[self.addy_1, False], [self.addy_2, False], 
                    [self.addy_3, False], [self.addy_4, False]]

        self.epsilon = 0.05

        self.brakes = False # pause the PID

        self.posteriors = []

        self.state_pub = rospy.Publisher("state", Float64, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            "scan_angle", Float64, self.scan_callback, queue_size=1
        )
        self.cmd_sub = rospy.Subscriber("cmd_vel_noisy", Twist, self.cmd_callback)


    def correction(self, time_elapsed):
        return time_elapsed * 0.01

    def cmd_callback(self, cmd_msg):

        global x_plot

        self.u = cmd_msg.linear.x

        # integrate velocity over discrete time to estimate robot position
        # self.x += self.u * 0.1

        print("CUR POS:", self.x)

        # TODO: enable me again
        # if we are at a addy, we stop for 2s
        for i, addy in enumerate(self.addy):
            if abs(self.x - addy[0]) < self.epsilon and addy[1] == False:
                addy[1] = True

                print("At addy", i, "sleeping...")
                # Observe the error (visually)

                # pause 
                self.brakes = True
                rospy.sleep(2.0)
                x_plot.extend(int(2.0 / 0.1) * [self.x])
                meas.extend(int(2.0 / 0.1) * [self.phi])
                cov.extend(int(2.0/0.1) * [self.P])
                self.brakes = False



    ## updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, msg):
        self.phi = msg.data

    ## call within run_kf to update the state with the measurement
    def predict(self, u=0):
        """
        TODO: update state via the motion model, and update the covariance with the process noise
        """
        return

    ## call within run_kf to update the state with the measurement
    def measurement_update(self):
        """
        TODO: update state when a new measurement has arrived using this function
        """
        return

    def run_kf(self):
        current_input = self.u
        # current_input = 0.07
        current_measurement = self.phi
        # print(current_measurement)

        """
        TODO: complete this function to update the state with current_input and current_measurement
        """

        global x_plot
        global meas
        global cov

        y_cn = 0.60
        x_cn = 0.60 * 3
        dt = 0.1

        A_k = 1
        B_k = dt

        D_k = lambda x_k : y_cn / ((x_cn - x_k) ** 2 + y_cn ** 2)
        h = lambda x_k : math.atan(y_cn/(x_cn - x_k))
        f = lambda x_k : x_k + dt * current_input                                                                                                                                  * 0.685
        if np.isnan(current_measurement):
            current_measurement = self.prev_phi
        else:
            self.prev_phi = self.phi
        # state estimation
        x_prior = f(self.x)

        # covariance estimation
        P_prior = A_k * self.P * A_k + self.Q
        S = D_k(x_prior) * P_prior * D_k(x_prior) + self.R
        # print(P_prior, D_k(x_prior), S)
        print(S)
        W = P_prior * D_k(x_prior) / S
        self.P = P_prior - W * S * W

        # measurement estimation and state update
        z_prior = h(x_prior)
        s = current_measurement - z_prior
        print(W)
        #W = 0
        self.x = x_prior + W*s
        assert np.isnan(S) == False
        assert np.isnan(W) == False
        assert np.isnan(current_measurement) == False
        assert np.isnan(self.x) == False

        # publish the state
        # self.posteriors.append(posterior)
        x_plot.append(self.x)
        meas.append(current_measurement)
        cov.append(self.P)
        self.state_pub.publish(self.x)
        


    def camera_callback(self, msg):
        """Callback for line index."""
        # access the value using msg.data
        self.index = msg.data


    def PID(self):

        if self.brakes:
            return
        
        if self.index == -1:
            time.sleep(0.1)
        
        k_p = 0.001
        k_i = 0.0
        k_d = 0.0025

        desired = 320
        integral = 0.0
        derivative = 0.0
        lasterror = 0.0

        actual = self.index

        error = desired - actual
        integral += error
        derivative = error - lasterror
        # publish the twist message
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = k_p * error + k_i * integral + k_d * derivative
        self.cmd_pub.publish(twist)
        # rospy.loginfo(twist)
        lasterror = error
        self.rate.sleep()



if __name__ == "__main__":
    rospy.init_node("lab4")

    h = 0.60  # y distance to tower
    d = 0.60 * 3  # x distance to tower (from origin)

    x_0 = 0  # initial state position

    Q = 0.001*2  # TODO: Set process noise covariance
    R = 5**2  # TODO: measurement noise covariance
    P_0 = 1  # TODO: Set initial state covariance

    kf = KalmanFilter(h, d, x_0, Q, R, P_0)
    rospy.sleep(1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        kf.run_kf()
        kf.PID()
        rate.sleep()
    print("PLOTTING")
    x = np.array(x_plot)
    fig, axs = plt.subplots(1, 3)
    time = np.arange(x.shape[0])*0.1
    axs[0].plot(time, x, label="position", color="blue")
    axs[0].legend()
    axs[1].plot(time, np.array(meas), label="angle", color="orange")
    axs[0].set_ylim(-0.2, 5)
    axs[1].legend()
    axs[2].plot(time, np.array(cov), label="covariance")
    axs[2].legend()
    axs[0].grid()
    axs[1].grid()
    axs[2].grid()
    plt.show()
