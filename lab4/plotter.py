#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64
from std_msgs.msg import UInt32

import matplotlib.pyplot as plt
import math
import numpy as np
import time

x_plot = []

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global x_plot
    x_plot.append(data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", Float64, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except Exception as e:
        # plot x_plot        
        import matplotlib.pyplot as plt
        x = np.array(x_plot)
        time = np.arange(x.shape[0])*0.1
        plt.plot(time, x)
        plt.grid()
        plt.show()

