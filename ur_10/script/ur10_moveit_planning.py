#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def main():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur_moveit_planning', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "ur_10"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # HomePose
    initial_pose = move_group.get_current_joint_values()
    initial_pose[0] = -1.229  
    initial_pose[1] = 4.719
    initial_pose[2] = -0.042  
    initial_pose[3] = 4.831  
    initial_pose[4] = 1.480  
    initial_pose[5] = 0.000

    move_group.set_joint_value_target(initial_pose)
    move_group.plan()
    move_group.go(wait=True)
    rospy.loginfo("HomePose reached")
    move_group.stop()
    move_group.clear_pose_targets()

    # PTP
    joint_goal = move_group.get_current_joint_values()

    #PrePose
    joint_goal[0] = -1.061
    joint_goal[1] = 4.691
    joint_goal[2] = -1.647
    joint_goal[3] = 4.831 
    joint_goal[4] = 1.480  
    joint_goal[5] = 0.000

    move_group.set_joint_value_target(joint_goal)
    plan = move_group.plan()

    if plan:
        move_group.go(wait=True)
        rospy.loginfo("PrePose reached")
    else:
        rospy.logwarn("Plan was not successful. Collision detected.")

    move_group.stop()
    move_group.clear_pose_targets()

    # LIN
    waypoints = []
    wpose = move_group.get_current_pose().pose

    #PickPose
    wpose.position.x = 0.0  
    waypoints.append(wpose)

    wpose.position.y += 0.15  
    waypoints.append(wpose)

    wpose.position.z -= 0.3 
    waypoints.append(wpose)

    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  

    if fraction == 1.0:
        move_group.execute(plan, wait=True)
        rospy.loginfo("PickPose reached")
    else:
        rospy.logwarn("Cartesian path was not fully planned. Collision detected.")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
