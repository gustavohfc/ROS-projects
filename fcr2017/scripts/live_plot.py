#!/usr/bin/env python
# pylint: disable=C0103

import numpy as np
from std_msgs.msg import Float64MultiArray
import rospy
import matplotlib.pyplot as plt

names = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R']
positions = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]


def write_data(data):
    f.write(str(rospy.get_time()) + '\n')
    for i in range(18):
        f.write('P(S = ' + names[i] + '): ' + str(data.data[i]) + '\n')
    f.write('\n\n')


def callback(data):
    # for i in range(18):
    #     print(names[i] + ': ' + str(data.data[i]))

    # print('\n\n\n\n')

    write_data(data)

    plt.clf()
    plt.axis([0, 18, 0, 1])
    plt.bar(positions, data.data)
    plt.xticks(positions, names)

    plt.xlabel('Node (S)')
    plt.ylabel('Probability P(S)')

    plt.pause(0.05)


if __name__ == "__main__":
    rospy.init_node('live_plot', anonymous=True)
    rospy.Subscriber("probabilities", Float64MultiArray, callback, queue_size=1)

    f = open('Probability.txt', 'w')

    plt.ion()
    plt.show(block=True)

    rospy.spin()
