#!/usr/bin/env python
import argparse
import rospy
import baxter_interface
import sys
sys.path.append('/home/robot/ros_baxter_1.2.0/src/baxter_examples/scripts')
import joint_trajectory_client
import math
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)

def getPoseMsg(pose):
	pos = pose['position']
	ori = pose['orientation']
	poseMsg = Pose(
		position = Point(
			x = pos.x,
			y = pos.y,
			z = pos.z
			),
		orientation = Quaternion(
			x = ori.x,
			y = ori.y,
			z = ori.z,
			w = ori.w
			)
		)
	return poseMsg

def getPoseStamped(pose):
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	poseMsg = getPoseMsg(pose)
	psStamped = PoseStamped(header=hdr, pose=poseMsg)
	return psStamped

def getPoseTrajectory(pose0, pose1, duration, interval = 0.01):
	"""
	Calculate a pose0-to-pose1 trajectory whose points are denoted by poses
	Input: 
		pose0 and pose1 is got from Limb.endpoint_pose()
		duration is the time you want to execute the trajectory
		interval is the time interval between every two points 
	"""
	pos0 = pose0['position']
	ori0 = pose0['orientation']
	pos1 = pose1['position']
	ori1 = pose1['orientation']
	theta0 = 2*math.acos(ori0.w)
	theta1 = 2*math.acos(ori1.w)
	sin0 = math.sin(theta0/2)
	sin1 = math.sin(theta1/2)
	orii0 = ori0.x/sin0
	orij0 = ori0.y/sin0
	orik0 = ori0.z/sin0
	orii1 = ori1.x/sin1
	orij1 = ori1.y/sin1
	orik1 = ori1.z/sin1

	point_number = int(duration / interval)

	poseTrajectory = []

	for i in range(point_number):
		posx = ((point_number-i)*pos0.x + i*pos1.x)/point_number
		posy = ((point_number-i)*pos0.y + i*pos1.y)/point_number
		posz = ((point_number-i)*pos0.z + i*pos1.z)/point_number
		theta = ((point_number-i)*theta0 + i*theta1)/point_number
		sin_theta_by_2 = math.sin(theta/2)
		orix = ((point_number-i)*orii0 + i*orii1)/point_number * sin_theta_by_2
		oriy = ((point_number-i)*orij0 + i*orij1)/point_number * sin_theta_by_2
		oriz = ((point_number-i)*orik0 + i*orik1)/point_number * sin_theta_by_2
		oriw = math.cos(theta/2)

		poseMsg = Pose(position = Point(x = posx,y = posy,z = posz),
		orientation = Quaternion(x = orix,y = oriy,z = oriz,w = oriw))
		hdr = Header(stamp=rospy.Time.now(), frame_id = 'base')
		poseStamped = PoseStamped(header = hdr, pose = poseMsg)
		poseTrajectory.append(poseStamped)

	poseTrajectory.append(getPoseStamped(pose1))
	
	return poseTrajectory

def getJointAngleTrajectory(poseTrajectory, limb_name):
	"""
	Convert a pose trajectory to a joint angle trajectory using ik calculation 
	"""
	ns = "ExternalTools/" + limb_name + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	ikreq.pose_stamp += poseTrajectory

	try:
		rospy.wait_for_service(ns, 5.0)
		resp = iksvc(ikreq)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return None

	jointAngleTrajectory = []
	point_number = len(poseTrajectory)
	for i in range(point_number):
		if (resp.isValid[i]):
			jointAngleTrajectory.append(resp.joints[i].position)
		else:
			print("invalid pose: ", poseTrajectory[i].pose)
			break
	return jointAngleTrajectory

if __name__ == '__main__':
	"""
	Author: Troye Fang
	Intro: The programme is used to calculate a smooth trajectory from given pose0 to 
	pose1, then execute the trajectory.
	Usage: First run joint_trajectory_server, then run this node	
	"""
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt)
	parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
	args = parser.parse_args(rospy.myargv()[1:])

	rospy.init_node('smooth_move')

	limb_name = args.limb
	limb = baxter_interface.Limb(limb_name)
	trajectory = joint_trajectory_client.Trajectory(limb_name)
	rospy.on_shutdown(trajectory.stop)

	pose0 = limb.endpoint_pose()
	posx = input('posx: ')
	posy = input('posy: ')
	posz = input('posz: ')
	orix = pose0['orientation'].x
	oriy = pose0['orientation'].y
	oriz = pose0['orientation'].z
	oriw = pose0['orientation'].w
	duration = input('input the duration seconds to execute the trajectory: \n')

	pose1 = {'position': limb.Point(posx, posy, posz), 'orientation': limb.Quaternion(orix, oriy, oriz, oriw)}

	poseTrajectory = getPoseTrajectory(pose0, pose1, duration, 0.2)
	jointAngleTrajectory = getJointAngleTrajectory(poseTrajectory, limb_name)

	for j in jointAngleTrajectory:
		print(j)

	current_angles = [limb.joint_angle(joint) for joint in limb.joint_names()]
	point_number = len(jointAngleTrajectory)
	trajectory.add_point(current_angles, 0.0)
	for i in range(point_number):
		trajectory.add_point(jointAngleTrajectory[i], 5 + i*0.2)

	print("Start moving")
	trajectory.start()
	trajectory.wait(15.0)
	print("Exiting")


