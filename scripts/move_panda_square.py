#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
from test.msg import size

def callback(data):
	## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
	## kinematic model and the robot's current joint states
	robot = moveit_commander.RobotCommander()

	## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
	## for getting, setting, and updating the robot's internal understanding of the
	## surrounding world:
	scene = moveit_commander.PlanningSceneInterface()

	## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
	## to a planning group (group of joints). 
	group_name = 'panda_arm'
	group = moveit_commander.MoveGroupCommander(group_name)

	## Create a `DisplayTrajectory`_ ROS publisher which is used to display
	## trajectories in Rviz:
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                   moveit_msgs.msg.DisplayTrajectory,
		                                   queue_size=20)
	## publishing initialised
        ## Planning to a Joint Goal
        ## We can get the joint values from the group and adjust some of the values:
        print '--------------------------------------------------'
        print ' Move Panda - Received square size = %s ' % data.size
        print '--------------------------------------------------'
        print '--------------------------------------------------'
        print ' Move Panda - Going to Starting Configuration'
        print '--------------------------------------------------'
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = 0
	joint_goal[1] = -pi/4
	joint_goal[2] = 0
	joint_goal[3] = -pi/2
	joint_goal[4] = 0
	joint_goal[5] = pi/3
	joint_goal[6] = 0

	# The go command can be called with joint values, poses, or without any
	# parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()
        print '--------------------------------------------------'
        print ' Move Panda - Planning Motion Trajectory '
        print '--------------------------------------------------'
        # initialise array of positions
        #plan a Cartesian path directly by specifying a list of waypoints
        waypoints = []
        wpose = group.get_current_pose().pose

        wpose.position.x += data.size            # First move forward (x)  
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += data.size            # Second move forward (y) 
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x -= data.size            # Third move backwards (x) 
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= data.size            # Last move backwards (y)
        waypoints.append(copy.deepcopy(wpose))

	# We want the Cartesian path to be interpolated at a resolution of 1 cm
	# which is why we will specify 0.01 as the eef_step in Cartesian
	# translation.  We will disable the jump threshold by setting it to 0.0,
	# ignoring the check for infeasible jumps in joint space.
        (plan, fraction) = group.compute_cartesian_path(
                                             waypoints, # waypoints to follow
                                             0.01,      # eef_step
                                             0.0)       # jump_threshold


	## Displaying a Trajectory
	## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
	## We populate the trajectory_start with our current robot state to copy over
	## any AttachedCollisionObjects and add our plan to the trajectory.

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(5)


        # Showing Planned Trajectory
        print '--------------------------------------------------'
        print ' Move Panda - Showing Planned Trajectory '
        print '--------------------------------------------------'
	# Publish
        display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(5)
        # Execute planned trajectory
        print '--------------------------------------------------'
        print ' Move Panda - Executing Planned Trajectory '
        print '--------------------------------------------------'
        group.execute(plan, wait=True)
	print '--------------------------------------------------'
	print ' Move Panda - Waiting for desired size of square '
	print '--------------------------------------------------'


def move_panda_square():
	# Initialize `moveit_commander`_ and a `rospy`_ node:
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_panda_square', anonymous=True)

	# subscribe the square_size and send data to callback
	print '--------------------------------------------------'
	print ' Move Panda - Waiting for desired size of square '
	print '--------------------------------------------------'
	rospy.Subscriber('size', size, callback)
	rospy.spin()


if __name__ == "__main__":
    try:
   	 move_panda_square()
    except rospy.ROSInterruptException:
        pass

