#!/usr/bin/env python
# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
import traceback
import tf
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
# trac_ik import for transfrom from task space to joint
from trac_ik_python.trac_ik import IK
# useful for quaternion math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

try:
  from math import pi, tau, dist, fabs, cos
except: # For Python 2 compatibility
  from math import pi, fabs, cos, sqrt
  tau = 2.0*pi
  def dist(p, q):
    return sqrt(sum((p_i - q_i)**2.0 for p_i, q_i in zip(p,q)))
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def main():
    global move_group, ik_solver, robot, scene, allPoses
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('open_manipulator_nav', anonymous=True)
    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    init_pose = move_group.get_current_pose().pose
    allPoses = []
    ik_solver = IK("link1", "link8")

    rospy.init_node("open_manipulator_nav", anonymous=True)
    rospy.Subscriber("/pose_array", geometry_msgs.msg.PoseArray, callback)
    rospy.spin()

def callback(data):
    global allPoses
    # want to find poses that are in data but not in allPoses
    for pose in data.poses:
        if pose in allPoses:
            pass
        else:
            allPoses.append(pose)
            go_to_pose_goal(pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
            pose.orientation.y, pose.orientation.z, pose.orientation.w)
            # end go_to_pose_goal statement


def go_to_pose_goal(x, y, z, ox, oy, oz, ow):
    # Copy class variables to local variables to make the web tuyorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    ## BEGIN_SUB_TUTORIAL plan_to_pose
    global move_group, ik_solver
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    #pose_goal = geometry_msgs.msg.Pose()
    seed_state = [0.0] * ik_solver.number_of_joints # should be a matrix of N joints
    joint_goal = ik_solver.get_ik(seed_state, x, y, z, ox, oy, oz, ow)
    ## Now, we call the planner to compute the plan and execute it.
    move_group.go(joint_goal, wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # this is useful for checking for deviations
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = ow
    pose_goal.orientation.x = ox
    pose_goal.orientation.y = oy
    pose_goal.orientation.z = oz
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    current_pose = move_group.get_current_pose().pose

    return all_close(pose_goal, current_pose, 0.01)


def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


if __name__=="__main__":
    try:
        main()
