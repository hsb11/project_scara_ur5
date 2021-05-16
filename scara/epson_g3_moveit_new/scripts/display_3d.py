#! /usr/bin/env python
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
import actionlib
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import os
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import time

l1 = []
l2 = []
l3 = []
i = 0


def draw(points):
	global l1, l2, l3, i
	# if i == 0:
	# 	plt.show()
	# 	axes = plt.gca()
	# 	axes.set_xlim(-0.25, 0.25)
	# 	axes.set_ylim(-0.25, 0.25)
	# 	line, = axes.plot(l1, l2, 'r-')
	# 	i = 1
	l1.append(points[0])
	l2.append(points[1])
	l3.append(points[2])
	plt.ylim(-0.3, 0.3)
	plt.xlim(-0.3, 0.3)
	plt.xlabel('x - axis')
	plt.ylabel('y - axis')
	plt.title('Character')
	plt.plot(l1, l2)
	# plt.pause(0.05)
	plt.show()
	# line.set_xdata(l1)
	# line.set_ydata(l2)
	# plt.draw()
	# plt.pause(1e-17)
	# time.sleep(0.1)


def main():
    rospy.init_node('display', anonymous=True)

    rate = rospy.Rate(20)
    listener = tf.TransformListener ()

    s1 = []
    s2 = []
    s3 = []

    # mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_title('Tip Position\n', fontsize="12")

    trans = [0, 0, 0]
    while not rospy.is_shutdown():
    	try:
            (trans, rot) = listener.lookupTransform('world', 'link_3', rospy.Time(0))
            # draw(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # if trans[2] - 0.1 <= 0.005:
        s1.append(trans[0])
        s2.append(trans[1])
        s3.append(trans[2])

        # plt.ylim(-0.3, 0.3)
        # plt.xlim(-0.3, 0.3)
        # plt.zlim(0, 0.3)
        # plt.xlabel('x - axis')
        # plt.ylabel('y - axis')
        ax.set_xlim(-0.3, 0.3)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(0, 0.3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # plt.zlabel('z - axis')
        # plt.title('Tip Position')
        # plt.plot(s1, s2)
        ax.plot(s1, s2, s3)
        # ax.legend()
        plt.pause(0.05)
        if trans == [0.10, -0.05, 0.1]:
	        plt.show()
    	rate.sleep()


# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass