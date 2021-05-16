#! /usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener
import os


class ScaraMoveit:

    # Constructor
    def __init__(self):

        rospy.init_node('character', anonymous=True)

        self._planning_group = "scara_position"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        self._curr_state = self._robot.get_current_state()

        # rospy.loginfo(self._planning_frame)
        # rospy.loginfo(self._eef_link)
        # rospy.loginfo(self._group_names)
        # rospy.loginfo(self._curr_state)

    def goal(self, w):
        a1 = 0.12
        a2 = 0.13
        num = w[0]*w[0] + w[1]*w[1] - a1*a1 - a2*a2
        den = 2*a1*a2
        rospy.loginfo(num/den)
        # rospy.loginfo(den)
        q2 = math.acos(num/den)
        num = a2*math.sin(q2)*w[0] + (a1 + a2*math.cos(q2))*w[1]
        den = (a1 + a2*math.cos(q2))*w[0] - a2*math.sin(q2)*w[1]
        q1 = math.atan2(num, den)
        q3 = w[2] - 0.129
        q4 = 0
        # rospy.loginfo(q1)
        # rospy.loginfo(q2)
        joint_goal = self._group.get_current_joint_values()
        joint_goal = [q1, -q2, q3, q4]
        self._group.go(joint_goal, wait=True)

    def go_to_pose(self, arg_pose):
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        return flag_plan

    def clear(self):
        self._group.clear_pose_targets()

    def go_to_named_pose(self, arg_pose):
        self._group.set_named_target(arg_pose)
        self._group.go(wait=True)

    def test(self):
        self._group.setPositionTarget(0.25, 0, 0.112, "link_3")

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()


def main():
    scara = ScaraMoveit()
    # scara_pose_1 = geometry_msgs.msg.Pose()

    # Home position
    # scara.go_to_named_pose("home")
    # scara.test()
    ########## H character 2d
    # w = [0.20, 0, 0.1, 0, 0, -1]
    # scara.goal(w)

    # w = [0.10, 0, 0.1, 0, 0, -1]
    # scara.goal(w)

    # w = [0.15, 0, 0.1, 0, 0, -1]
    # scara.goal(w)

    # w = [0.15, -0.05, 0.1, 0, 0, -1]
    # scara.goal(w)

    # w = [0.20, -0.05, 0.1, 0, 0, -1]
    # scara.goal(w)

    # w = [0.15, -0.05, 0.1, 0, 0, -1]
    # scara.goal(w)

    # w = [0.10, -0.05, 0.1, 0, 0, -1]
    # scara.goal(w)

    ##########################

    w = [0.16, 0, 0, 0, 0, -1]
    scara.goal(w)

    w = [0, -0.16, 0.1, 0, 0, -1]
    scara.goal(w)

    w = [-0.16, 0, 0.129, 0, 0, -1]
    scara.goal(w)

    w = [-0.12, 0.16, 0.129, 0, 0, -1]
    scara.goal(w)

    w = [0, 0.16, 0.1, 0, 0, -1]
    scara.goal(w)

    w = [0.16, 0, 0, 0, 0, -1]
    scara.goal(w)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
