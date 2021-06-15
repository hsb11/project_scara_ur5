#! /usr/bin/env python
import rospy
import tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

#  This script is used to plot the tool tip position using matplotlib

l1 = []
l2 = []
l3 = []
i = 0


def draw(points):
	global l1, l2, l3, i
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


def main():
    rospy.init_node('display', anonymous=True)

    rate = rospy.Rate(20)
    listener = tf.TransformListener ()

    s1 = []
    s2 = []
    s3 = []

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_title('Tip Position\n', fontsize="12")

    trans = [0, 0, 0]
    while not rospy.is_shutdown():
    	try:
            (trans, rot) = listener.lookupTransform('world', 'link_3', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        s1.append(trans[0])
        s2.append(trans[1])
        s3.append(trans[2])

        ax.set_xlim(-0.3, 0.3)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(0, 0.3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.plot(s1, s2, s3)
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