import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from rtabmap_ros.srv import *
from std_srvs.srv import Empty

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('arm_nav_controller', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
clear_rtab = rospy.ServiceProxy('/rtabmap/reset', Empty)
clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
set_odom_pose = rospy.ServiceProxy('/rtabmap/reset_odom_to_pose', ResetPose)
listener = tf.TransformListener()

init_joints = [0, 0, -0.5, -1.18, 0, 1.15, 0] #pose where arm does not sag under weight (camera position is correct)

move_group.go(init_joints, wait=True) #Move to initialization pose
move_group.stop()
rospy.sleep(3.) #Wait 3 seconds for arm to settle
(trans,rot) = listener.lookupTransform('/world', 'fk_camera_link', rospy.Time(0))
rpy = euler_from_quaternion(rot)
print rpy
set_odom_pose(trans[0], trans[1], trans[2], rpy[0], rpy[1], rpy[2]) #set camera_link to match fk_camera_link for proper map alignment
clear_rtab() #Clear the rtabmap map
clear_octomap() #Clear the moveit occupancy grid

#begin scanning here