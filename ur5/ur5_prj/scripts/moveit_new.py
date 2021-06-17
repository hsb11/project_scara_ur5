#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf
from tf.transformations import quaternion_from_euler
from tf import TransformListener
from geometry_msgs.msg import Pose

#  This script is responsible for detecting, following and picking up the ball


class Ur5Moveit():

    # Constructor
    def __init__(self):

        rospy.init_node('moveit_perc', anonymous=True)

        self.ur5_pose_1 = geometry_msgs.msg.Pose()
        self.t = TransformListener()

        self._planning_group = "ur5_arm_plangrp"
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

        self._gripper = "gripper_plangrp"
        self._hand_group = moveit_commander.MoveGroupCommander(self._gripper)

        self.goal_pose = Pose()

    def go_to_pose(self, arg_pose):
        print('Going to Goal')
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        self.got_pose_goal = 0
        return flag_plan

    def clear(self):
        self._group.clear_pose_targets()

    def go_to_named_pose(self, arg_pose):
        self._group.set_named_target(arg_pose)
        self._group.go(wait=True)
        self.got_named_goal = 0

    def detect(self):
        pass

    def pick(self):#, arg):
        self._hand_group.set_named_target(arg)
        self._hand_group.go(wait=True)

    def place(self):
        self._hand_group.set_named_target("open")
        self._hand_group.go(wait=True)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()


def main():
    ur5 = Ur5Moveit()
    print('Move to Start')
    ur5.go_to_named_pose('start2')
    print('Reached Starting Position now starting to detect')
    while not rospy.is_shutdown():
        obj_ids = [4, 5, 6]
        for i in obj_ids:
            if ur5.t.frameExists("object_"+str(i)):
                try:
                    (ur5.goal_pose, rotation) = ur5.t.lookupTransform("world", "object_"+str(i), rospy.Time(0))
                    print(ur5.goal_pose)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                ur5.ur5_pose_1.position.x = ur5.goal_pose[0]
                ur5.ur5_pose_1.position.y = ur5.goal_pose[1] - 0.15
                ur5.ur5_pose_1.position.z = 0.80  # ur5.goal_pose[2] + 0.15    # 0.2
                q1 = quaternion_from_euler(+3.14/4, +3.14, 0)
                ur5.ur5_pose_1.orientation.x = q1[0]
                ur5.ur5_pose_1.orientation.y = q1[1]
                ur5.ur5_pose_1.orientation.z = q1[2]
                ur5.ur5_pose_1.orientation.w = q1[3]
                print('Moving towards object')
                ur5.go_to_pose(ur5.ur5_pose_1)
    rospy.sleep(0.2)


if __name__ == '__main__':
    main()
