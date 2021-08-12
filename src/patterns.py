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
    global move_group, robot, scene
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
    print("Initial Pose : \n -----------------------------------")
    print(init_pose)
    inp = input("Sweep? [y/n/L] :").lower()
    if inp == "y":
        n = int(input("Enter n : "))
        x = float(input("Enter x : "))
        y = float(input("Enter y : "))
        z = float(input("Enter z : "))
        rectangle_sweep(n, x, y, z)
    elif inp == "n":
    	x = float(input("Enter x : "))
    	y = float(input("Enter y : "))
    	z = float(input("Enter z : "))
    	rectangle(x, y, z)
    else:
        n = int(input("Enter n : "))
        x = float(input("Enter x : "))
    	y = float(input("Enter y : "))
    	z = float(input("Enter z : "))
        rectangle_corner_sweep(n, x, y, z)

## NOTE: All paths assume that the bot starts from the home position
#xyz = forwards, left, up
# rectangle is four points, rectangle goes CW
def rectangle(dx=0, dy=0.15, dz=0.15):
    global move_group
    init_pose = move_group.get_current_pose().pose
    ix = init_pose.position.x
    iy = init_pose.position.y
    iz = init_pose.position.z

    iox = init_pose.orientation.x
    ioy = init_pose.orientation.y
    ioz = init_pose.orientation.z
    iow = init_pose.orientation.w
    #p1
    go_to_pose_goal(ix+dx, iy+dy, iz, iox, ioy, ioz, iow)
    print("p1")
    #p2
    go_to_pose_goal(ix+dx, iy-dy, iz, iox, ioy, ioz, iow)
    print("p2")
    #p3
    go_to_pose_goal(ix+dx, iy-dy, iz-2*dz, iox, ioy, ioz, iow)
    print("p3")
    #p4
    go_to_pose_goal(ix+dx, iy+dy, iz-2*dz, iox, ioy, ioz, iow)
    print("p4")
    #center
    go_to_pose_goal(ix, iy, iz, iox, ioy, ioz, iow)


# this path follows a rectangle corner-wise, but sweeps horizontally every n lines incrementally (vertically)
def rectangle_sweep(n=5, dx=0, dy=0.1, dz=0.08):
    global move_group
    init_pose = move_group.get_current_pose().pose
    ix = init_pose.position.x
    iy = init_pose.position.y
    iz = init_pose.position.z

    iox = init_pose.orientation.x
    ioy = init_pose.orientation.y
    ioz = init_pose.orientation.z
    iow = init_pose.orientation.w
    for c in range(n):
    	go_to_pose_goal(ix+dx, iy+dy, iz-float(c)/float(n)*dz, iox, ioy, ioz, iow)
        go_to_pose_goal(ix+dx, iy-dy, iz-float(c)/float(n)*dz, iox, ioy, ioz, iow)
    # center after for loop
    go_to_pose_goal(ix, iy, iz, iox, ioy, ioz, iow)

# for a given rectangle, we want to be able to sweep around corners.
def rectangle_corner_sweep(n=5, dx=0, dy=0.1, dz=0.1):
    global move_group
    init_pose = move_group.get_current_pose().pose
    ix = init_pose.position.x
    iy = init_pose.position.y
    iz = init_pose.position.z

    iox = init_pose.orientation.x
    ioy = init_pose.orientation.y
    ioz = init_pose.orientation.z
    iow = init_pose.orientation.w
    (iroll, ipitch, iyaw) = euler_from_quaternion([iox, ioy, ioz, iow])
    py = quaternion_from_euler(iroll, ipitch, iyaw-pi/2) # positive y quaternion, head facing in negative y direction
    ny = quaternion_from_euler(iroll, ipitch, iyaw+pi/2) # negative y quaternion, head facing in positive y direction
    print(init_pose)
    print(py)
    print(ny)
    for c in range(n):
        go_to_pose_goal(ix+dx, iy+dy, iz-float(c)/float(n)*dz, py[0], py[1], py[2], py[3])
    	go_to_pose_goal(ix+dx, iy+dy, iz-float(c)/float(n)*dz, iox, ioy, ioz, iow)
    	go_to_pose_goal(ix+dx, iy-dy, iz-float(c)/float(n)*dz, iox, ioy, ioz, iow)
        go_to_pose_goal(ix+dx, iy-dy, iz-float(c)/float(n)*dz, ny[0], ny[1], ny[2], ny[3])
    go_to_pose_goal(ix, iy, iz, iox, ioy, ioz, iow)
def snake():
    pass

def spiral_outwards():
    pass

def spiral_inwards():
    pass

# assumes original quaternion orientation is preserved
#@param du: the minimum incremental difference
def go_along_line(x, y, z, du):
    global move_group
    init_pose = move_group.get_current_pose().pose
    ix = init_pose.position.x
    iy = init_pose.position.y
    iz = init_pose.position.z

    iox = init_pose.orientation.x
    ioy = init_pose.orientation.x
    ioz = init_pose.orientation.x
    iow = init_pose.orientation.x

    # current xyz
    cx = ix
    cy = iy
    cz = iz
    total_pose_count = max([abs((x-ix)/du), abs((y-iy)/du), abs((z-iz)/du)])
    print(total_pose_count)
    pose_count = 0
    # est incremental difference for each direction
    dx = (x - ix) / total_pose_count
    dy = (y - iy) / total_pose_count
    dz = (z - iz) / total_pose_count
    print(dx)
    print(dy)
    print(dz)
    # while not within range +-du of xyz
    # same as (cx > x + du or cx < x - du) or (cy > y + du or cy < y - du) or (cz > z + du or cz < z - du)
    while pose_count < total_pose_count:
        go_to_pose_goal(cx+dx, cy+dy, cz+dz, iox, ioy, ioz, iow)
        cx+=dx
        cy+=dy
        cz+=dz
        pose_count+=1

def go_to_pose_goal(x, y, z, ox, oy, oz, ow):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    ## BEGIN_SUB_TUTORIAL plan_to_pose
    global move_group
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    #pose_goal = geometry_msgs.msg.Pose()
    ik_solver = IK("link1", "link8") # initialize TRAC_IK solver
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
        while True:
            main()
    except rospy.ROSInterruptException:
        traceback.print_exc()
    except KeyboardInterrupt:
        exit()
