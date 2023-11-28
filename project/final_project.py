#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32, String, UInt32MultiArray, Float64MultiArray
import numpy as np
import colorsys

import time

class BayesLoc:
    def __init__(self, p0, colour_codes, colour_map):
        self.colour_sub = rospy.Subscriber(
            "mean_img_rgb", Float64MultiArray, self.colour_callback
        )
        self.line_sub = rospy.Subscriber("line_idx", UInt32, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(p0)
        self.colour_codes = colour_codes
        self.colour_map = colour_map
        self.probability = p0
        self.state_prediction = np.zeros(self.num_states)
        self.V = 0.04

        self.motors_off = False
        self.brakes = False

        self.colours = ["red", "green", "blue", "yellow", "line"]
        # self.hsv_ref = [colorsys.rgb_to_hsv(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0) for rgb in self.colour_codes]

        self.max_angular = 0.625 # rad/s
        self.colour_exit_max_angular = 0.15
        self.colour_exit_active_time = 0.75 # seconds
        self.last_state = "line"
        self.colour_exit_active = False
        self.tic = time.time()

        self.cur_colour = []  # most recent measured colour
        self.colour_count = 0
        self.rate = rospy.Rate(10)

    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        self.cur_colour = np.array(msg.data)  # [r, g, b]
        # print(self.cur_colour)

    def euclid(self, r, g, b, ref_rgb):
        return math.sqrt(2*(r - ref_rgb[0])**2 + (g - ref_rgb[1])**2 + (b - ref_rgb[2])**2)

    def dist(self, rgb):
        # weighted euclidean distance
        try:
            r, g, b = rgb
        except Exception as e:
            print(e)
            return "line"

        # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)

        min_euclid = 1e9
        min_colour = "line"

        for i, ref_rgb in enumerate(self.colour_codes):

            if abs(r - g) <= 15 and abs(g - b) <= 15 and abs(b - r) <= 15:
                return "line"
            # cur_euclid = math.sqrt((h - hsv[0])**2 + (s - hsv[1])**2 + (v-hsv[2])**2)
            # cur_euclid = math.sqrt(1.5*(r - ref_rgb[0])**2 + 3*(g - ref_rgb[1])**2 + 3*(b - ref_rgb[2])**2)

            cur_euclid = math.sqrt(2*(r - ref_rgb[0])**2 + (g - ref_rgb[1])**2 + (b - ref_rgb[2])**2)
            if cur_euclid < min_euclid:
                min_colour = self.colours[i]
                min_euclid = cur_euclid

        # find closest distance euclideanly
        error_red = self.euclid(r, g, b, self.colour_codes[0])
        error_green = self.euclid(r, g, b, self.colour_codes[1])
        error_blue = self.euclid(r, g, b, self.colour_codes[2])
        error_yellow = self.euclid(r, g, b, self.colour_codes[3])

        error_min = min(error_red, error_green, error_blue, error_yellow)

        if error_red == error_min:
            return "red"
        elif error_green == error_min:
            return "green"
        elif error_blue == error_min:
            return "blue"
        else:
            return "yellow"


        #print("VERDICT:", min_colour)
        return min_colour


    def detect_line(self, rgb):
        # sigma = 0.0
        # for x in rgb:
        #     sigma += x

        # if sigma < 425:
        #     return "line"
        return self.dist(rgb)

    def line_callback(self, msg):
        """
        TODO: Complete this with your line callback function from lab 3.
        """
        detection = self.detect_line(self.cur_colour)
        
        max_angular = self.max_angular

        if detection != "line":
            self.last_state = detection
            t = Twist()
            t.linear.x = self.V + 0.02
            if self.brakes:
                # twist = Twist()
                # twist.linear.x = 0
                # twist.angular.z = 0
                # # localizer.cmd_pub.publish(twist)
                # self.cmd_pub.publish(twist)

                for i in range(47):
                    t.linear.x = 0
                    t.angular.z = 0.35
                    self.cmd_pub.publish(t)
                    self.rate.sleep()

                for i in range(5):
                    t.linear.x = 0
                    t.angular.z = 0
                    self.cmd_pub.publish(t)
                    self.rate.sleep()

                for i in range(43):
                    t.linear.x = 0
                    t.angular.z = -0.35
                    self.cmd_pub.publish(t)
                    self.rate.sleep()

                for i in range(20):
                    t.linear.x = 0.03
                    t.angular.z = 0.0
                    self.cmd_pub.publish(t)
                    self.rate.sleep()

                self.brakes = False

            if not self.motors_off:
                self.cmd_pub.publish(t)
            self.rate.sleep()
            return
        else:
            # impose angular velocity constraints
            if self.last_state == detection and detection != "line":
                self.colour_count += 1
            else:
                self.colour_count = 0
            if self.last_state != "line" and self.colour_count == 5:
                # print("IMPOSING ANG CONSTRAINT:", self.colour_exit_max_angular)
                self.colour_exit_active = True
                self.tic = time.time()
                self.last_state = "line"
            if self.colour_exit_active and time.time() - self.tic > self.colour_exit_active_time:
                print(time.time() - self.tic)
                # turn off angular constraint
                # print("CANCEL ANG CONSTRAINT")
                self.colour_exit_active = False

        if self.colour_exit_active:
            max_angular = self.colour_exit_max_angular
        else:
            max_angular = self.max_angular
        # if msg.data == -1:
        #     time.sleep(0.1)
        
        k_p = 0.009
        k_i = 0.0
        k_d = 0.0035

        desired = 320
        integral = 0.0
        derivative = 0.0
        lasterror = 0.0

        actual = msg.data

        error = desired - actual
        integral += error
        derivative = error - lasterror

        # publish the twist message
        twist = Twist()
        twist.linear.x = self.V
        twist.angular.z = k_p * error + k_i * integral - k_d * derivative
        if twist.angular.z < 0 and twist.angular.z < -max_angular:
            twist.angular.z = -max_angular
        if twist.angular.z > 0 and twist.angular.z > max_angular:
            twist.angular.z = max_angular

        if self.brakes:
            # twist = Twist()
            # twist.linear.x = 0
            # twist.angular.z = 0
            # self.cmd_pub.publish(twist)

            twist = Twist()

            for i in range(47):
                twist.linear.x = 0
                twist.angular.z = 0.35
                self.cmd_pub.publish(twist)
                self.rate.sleep()

            for i in range(5):
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_pub.publish(twist)
                self.rate.sleep()

            for i in range(43):
                twist.linear.x = 0
                twist.angular.z = -0.35
                self.cmd_pub.publish(twist)
                self.rate.sleep()

            for i in range(20):
                twist.linear.x = 0.03
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.rate.sleep()

            self.brakes = False



        if not self.motors_off:
            self.cmd_pub.publish(twist)

        # print(twist)
        lasterror = error
        # self.rate.sleep()

        return

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_model(self, u):
        """
        State model: p(x_{k+1} | x_k, u)

        TODO: complete this function
        """
        # u = 0, 1
        # theres like no chance that if u say go forward it doesn't go forward
        return np.array([[0.0, 1, 0], [0.0, 0.0, 1.0]], dtype="float32")[u]

    def measurement_model(self, x):
        """
        Measurement model p(z_k | x_k = colour) - given the pixel intensity,
        what's the probability that of each possible colour z_k being observed?
        """
        if self.cur_colour is None:
            self.wait_for_colour()
        
        prob = np.zeros(len(self.colour_codes))

        # meas_model = np.array([[0.60, 0.20, 0.05, 0.05, 0.1],
        #                        [0.20, 0.60, 0.05, 0.05, 0.1],
        #                        [0.025, 0.025, 0.65, 0.20, 0.1],
        #                        [0.05, 0.05, 0.25, 0.60, 0.05],
        #                        [0.05, 0.1, 0.025, 0.025, 0.8]])
        meas_model = np.array([[0.60, 0.20, 0.05, 0.05],
                               [0.20, 0.60, 0.05, 0.05],
                               [0.025, 0.025, 0.65, 0.20],
                               [0.05, 0.05, 0.25, 0.60],
                               [0.05, 0.1, 0.025, 0.025]])

        colour_to_idx = {"blue": 0, "green": 1, "yellow": 2, "red": 3, "line": 4}
        # self.meas = meas_model[colour_to_idx[color]]
        # x should be the detect line output from the main method
        return meas_model[colour_to_idx[x]]

        """
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.cur_colour)
            and the reference RGB values of each colour (self.ColourCodes).
        """

        return prob

    def state_predict(self, u):
        n = len(self.colour_map)
        state_preds = np.zeros(n)
        state_model = self.state_model(u) # TODO drive command
        for j in range(n):
            s = 0.0
            for k in range(n):
                if j - k == 1 or j == 0 and k == n-1:
                    s += state_model[2] * self.probability[k]
                elif k - j == 1 or k == 0 and j == n-1: # drive backwards
                    s += state_model[0] * self.probability[k]
                elif j == k:
                    s += state_model[1] * self.probability[k]
            state_preds[j] = s
        return state_preds
        """
        TODO: Complete the state prediction function: update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """

    def state_update(self, x, u):
        print("updating state")
        print(x)

        n = len(self.colour_map)
        state_upd = np.zeros(n)
        meas = self.measurement_model(x) # TODO current measurement
        state_pred = self.state_predict(u)
        for j in range(n):
            state_upd[j] = meas[self.colour_map[j]] * state_pred[j]
        self.probability = state_upd / np.sum(state_upd)
        """
        TODO: Complete the state update function: update self.probabilities
        with the probability of being at each state
        """


if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: red, 1: green, 2: blue, 3: yellow, 4: line <-- wrong numberings prolly
    # current map starting at cell #2 and ending at cell #12
    # colour_map = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1]
    # blue, green, yellow, red, line

    #colour_to_idx = {"blue": 0, "green": 1, "yellow": 2, "red": 3, "line": 4}

    colour_map = [2, 1, 0, 3, 3, 1, 0, 3, 2, 1, 0]
    offices = [3, 7, 10]
    visited_offices = set()

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    # colour_codes = [
    #     [167, 146, 158],  # red
    #     [163, 184, 100],  # green
    #     [173, 166, 171],  # blue
    #     [167, 170, 117],  # yellow
    #     [150, 150, 150],  # line
    # ]

    # colour_codes = [
    #     [253.4693984375, 160.287484375, 116.329171875], # red
    #     [161.250515625, 175.643015625, 166.354953125], # green
    #     [185.0241640625, 169.0456015625, 191.5423203125], # blue
    #     [190.01903125, 169.7723125, 157.4255625], # yellow
    #     [150.980046875, 133.457421875, 140.574328125] # line
    # ]
    colour_codes = [
        # [251.130015625, 140.62609375, 89.67190625], # red
        [249.698890625, 160.8520078125, 102.7336953125],
        # [143.2434921875, 170.0344921875, 154.7638359375], # green
        [150.3832109375, 182.8780859375, 165.2038046875],
        [170.988671875, 149.003234375, 178.843015625], # blue
        # [182.1365234375, 159.4608671875, 147.0369296875], # yellow
        [187.37475, 183.7438125, 162.80165625],
        # [143.924296875, 128.9403046875, 135.141984375], # line
        [127.543125, 127.004125, 128.9620625]
    ]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)
    rospy.init_node("final_project")
    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.sleep(0.5)
    rate = rospy.Rate(10)
    most_likely = 0
    prev_x = "line"
    cnt = 0
    while not rospy.is_shutdown():
        """
        TODO: complete this main loop by calling functions from BayesLoc, and
        adding your own high level and low level planning + control logic
        """
        # wait for good consistent sensor reading
        if most_likely in offices and most_likely not in visited_offices and np.max(localizer.probability) > 0.55: # also must be more than 60% sure that ur actually there, will prob need to make at least one loop around the map
            u = 0
            visited_offices.add(most_likely)
            print("\n [ * ] Delivering MAIL to: \n", most_likely)
            print(localizer.probability)
            print(x)
            localizer.brakes = True
        else:
            u = 1

        x = localizer.detect_line(localizer.cur_colour)
        if prev_x == x and x != "line": # track how many ticks of an actual colour read it's giving
            cnt += 1
        elif x == "line":
            cnt = 0
        if cnt == 18 and not localizer.brakes:
            # if x is a colour for some cycles (8 in this case) do loc
            #could be lowered because the callback could tick many times before this counter can actually increment again
            # localizer.state_model(u) # drive
            # localizer.measurement_model(x) # reading

            if localizer.motors_off:
                print("Cur measurement: ", x)
                input()

            
            # localizer.state_predict(u)
            localizer.state_update(x, u)
            most_likely = np.argmax(localizer.probability)

            # if localizer.motors_off:
            #     print("Most likely state: ", most_likely)
            #     input()

            # cnt = 0

        rate.sleep()
        prev_x = x

    print("finished!")
    print(localizer.probability)
