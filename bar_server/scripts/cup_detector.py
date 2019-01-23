#!/usr/bin/env python

import cv2
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf
import baxter_interface
import copy
import struct
import math

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from std_msgs.msg import (
	Header,
	Empty,
)
from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)



class HandCamera:

	def __init__(self, color):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,self.callback)
		self.centerx = 0
		self.centery = 0
		self.color = color
		self.hsvColor = {
			'yellow': [[25, 100, 179], [40, 255, 255]],
			'pink': [[150, 0, 200], [170, 76, 255]],
		}

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			# rotate 180 degree
			cv_image = cv2.flip(cv_image, 0) 
			cv_image = cv2.flip(cv_image, 1)
		except CvBridgeError as e:
			print(e)
		# (rows,cols,channels) = cv_image.shape
		# if cols > 60 and rows > 60 :
		# 	cv2.circle(cv_image, (50,50), 10, 255)

		blured = cv2.blur(cv_image, (5, 5))
		hsv = cv2.cvtColor(blured, cv2.COLOR_BGR2HSV)

		lower = np.array(self.hsvColor[self.color][0]) # red
		upper = np.array(self.hsvColor[self.color][1])
		# lower_color= np.array([90, 43, 46])   # blue
		# upper_color = np.array([130, 255, 255])
		mask = cv2.inRange(hsv, lower, upper)
		tmp, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(cv_image, contours, -1, (0, 0, 255), 1)

		maxWidth = 0
		contIdx = -1
		for idx, c in enumerate(contours):
			rect = cv2.minAreaRect(c) 
			# return (minimum enclosing rectangle = [[centerx, centery], [width, height], rotate])
			if rect[1][0] > maxWidth:
				maxWidth = rect[1][0]
				contIdx = idx

		biggestRect = cv2.minAreaRect(contours[contIdx])
		self.centerx, self.centery = biggestRect[0]
		box = cv2.boxPoints(biggestRect)
		box = np.int0(box)
		cv2.drawContours(cv_image, [box], 0, (0, 0, 255))

		x0 = int(box[0, 0])
		y0 = int(box[0, 1])
		x1 = int(box[1, 0])
		y1 = int(box[1, 1])
		x2 = int(box[2, 0])
		y2 = int(box[2, 1])
		x3 = int(box[3, 0])
		y3 = int(box[3, 1])
		cor0 = ("{}:({}, {})".format(0, x0, y0))
		cor1 = ("{}:({}, {})".format(1, x1, y1))
		cor2 = ("{}:({}, {})".format(2, x2, y2))
		cor3 = ("{}:({}, {})".format(3, x3, y3))
		# cv2.putText(cv_image, cor0, (x0, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
		# cv2.putText(cv_image, cor1, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
		# cv2.putText(cv_image, cor2, (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
		# cv2.putText(cv_image, cor3, (x3, y3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

		cv2.namedWindow('Hand Image',cv2.WINDOW_AUTOSIZE)
		cv2.imshow("Hand Image", cv_image)
		cv2.waitKey(3)
#		try:
#			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#		except CvBridgeError as e:
#			print(e)
 	
 	def center(self):
 		return [self.centerx, self.centery]

class Grasper(object):
	def __init__(self, limb = 'right', verbose = True):
		self._verbose = verbose
		self._limb_name = limb
		self._limb = baxter_interface.Limb(limb)
		self._gripper = baxter_interface.Gripper(limb)
		ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		rospy.wait_for_service(ns, 5.0)
		init_joints = {'right_s0': -0.7954, 'right_s1': 0.3559, 'right_w0': -0.2320, 
			'right_w1': 1.2621, 'right_w2': 1.2498, 'right_e0': 1.4159, 'right_e1': 1.2908}
		self._limb.move_to_joint_positions(init_joints)
		self._gripper.open()
		# cv2.namedWindow('Kinect', cv2.WINDOW_NORMAL)


	def ik_request(self, pose):
		hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(PoseStamped(header = hdr, pose = pose))
		try:
			resp = self._iksvc(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e, ))
			return False
		resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
		limb_joints = {}
		if (resp_seeds[0] != resp.RESULT_INVALID):
			seed_str = {
				ikreq.SEED_USER: 'User Provided Seed',
				ikreq.SEED_CURRENT: 'Current Joint Angles',
				ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
			}.get(resp_seeds[0], 'None')
			if self._verbose:
				print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
					(seed_str)))

			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			if self._verbose:
				print("IK Joint Solution:\n{0}".format(limb_joints))
				print("----------------")
		else:
			rospy.logerr("INVALID POSE - No valid Joint Solution Found.")
			return False
		return limb_joints

	def _get_current_pose_msg(self):
		current_pose = self._limb.endpoint_pose()
		cur_pos = current_pose['position']
		cur_ori = current_pose['orientation']
		pose_msg = Pose()
		pose_msg.position.x = cur_pos.x
		pose_msg.position.y = cur_pos.y
		pose_msg.position.z = cur_pos.z
		pose_msg.orientation.x = cur_ori.x
		pose_msg.orientation.y = cur_ori.y
		pose_msg.orientation.z = cur_ori.z
		pose_msg.orientation.w = cur_ori.w
		return pose_msg

	def move_right(self, length):
		"""
		move to right-straight with angle of 60 degree
		length < 0 means moving to left-back
		"""
		goal_pose = self._get_current_pose_msg()
		goal_pose.position.x += length*math.sin(math.pi/3)
		goal_pose.position.y -= length*math.cos(math.pi/3)
		joint_angles = self.ik_request(goal_pose)
		if not joint_angles == False:
			self._limb.move_to_joint_positions(joint_angles)
		else:
			rospy.logerr('No Joint Angles Provided when moving right for %.2f.' % length)

	def move_straight(self, length):
		"""
		move to left-straight with angle of 30 degree
		"""
		goal_pose = self._get_current_pose_msg()
		goal_pose.position.x += length*math.sin(math.pi/6)
		goal_pose.position.y += length*math.cos(math.pi/6)
		joint_angles = self.ik_request(goal_pose)
		if not joint_angles == False:
			self._limb.move_to_joint_positions(joint_angles)
		else:
			rospy.logerr('No Joint Angles Provided when moving straight for %.2f.' % length)

	def move_up(self, length):
		"""
		move upward
		"""
		goal_pose = self._get_current_pose_msg()
		goal_pose.position.z += length
		joint_angles = self.ik_request(goal_pose)
		if not joint_angles == False:
			self._limb.move_to_joint_positions(joint_angles)
		else:
			rospy.logerr('No Joint Angles Provided when moving upward for %.2f.' % length)
			
	def move_to_position(self, x, y, z, orix = 0.6123, oriy = 0.3535, oriz = 0.6123, oriw = -0.3535):
		pose = Pose()
		pose.position.x = x
		pose.position.y = y
		pose.position.z = z
		pose.orientation.x = orix
		pose.orientation.y = oriy
		pose.orientation.z = oriz
		pose.orientation.w = oriw
		joint_angles = self.ik_request(pose)
		if not joint_angles == False:
			self._limb.move_to_joint_positions(joint_angles)
		else:
			rospy.logerr('No joint angles provided')

	def grasp(self):
		self._gripper.close()
		rospy.sleep(1)
		
	def release(self):
		self._gripper.open()
		rospy.sleep(1)
		
	def rotate(self, rel_radius):
		pos = self._limb.joint_angles()
		pos[self._limb_name + "_w2"] += rel_radius
		self._limb.move_to_joint_positions(pos)
		
	def move_to_neutral(self):
		self._limb.move_to_neutral()


class CupDetector:

	def __init__(self, color):
		self.bridge = CvBridge()
		self.centerx = 0
		self.centery = 0
		self.depth = 0
		self.depth_sub = rospy.Subscriber("/kinect2/hd/image_depth_rect",Image,self.callback2)
		self.image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect",Image,self.callback)

		self.color = color
		self.hsvColor = {
			'yellow': [[25, 100, 179], [40, 255, 255]],
			'pink': [[150, 0, 200], [170, 76, 255]],
			'purple': [[110, 43, 46], [155, 255, 255]],
			'blue': [[50, 56, 179], [92, 102, 224]],
			'orange': [[4, 128, 140], [50, 173, 199]]}
		

		# tf_listener = tf.TransformListener()
		# self.tf_base2kinect = self.lookupTransform(tf.TransformListener(), '/base', 'kinect2_link')
		self.tf_base2kinect = np.array(
			[[-0.99701393,  0.01709224, -0.07530658,  0.74413355],
			[ 0.07443345, -0.04698567, -0.99611847,  1.03124663],
			[-0.02056422, -0.99874932,  0.04557314,  0.19888489],
			[ 0.        , 0.         , 0.         , 1.        ]])

		self.cx=9.7160177704855255e+02
		self.cy=4.9317158047771312e+02
		self.fx=1.0616393051426160e+03
		self.fy=1.0651432447969651e+03

		# coordinate of reconized object in /base frame 
		self.objectx = 0
		self.objecty = 0
		self.objectz = 0

	def lookupTransform(self, tf_listener, target, source):
	    tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))

	    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
	    euler = tf.transformations.euler_from_quaternion(rot)

	    source_target = tf.transformations.compose_matrix(translate = trans, angles = euler)
	    return source_target

	def pixel2cam(self):
		d=self.depth/1000.0;
		x=-(self.cx-self.centerx)*d/self.fx
		y=-(self.cy-self.centery)*d/self.fy
		return [x,y,d]

	def callback2(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
		except CvBridgeError as e:
			print(e)
			return

		try:
			# print ('depth: ', cv_image[int(self.centery)][int(self.centerx)])
			self.depth=cv_image[int(self.centery)][int(self.centerx)]
			x, y, z = self.pixel2cam()
			if z > 0.5 and z < 2:	# valid data
				coord_in_kinect = np.array([x, y, z, 1])
				pnt = self.tf_base2kinect.dot(coord_in_kinect)
				[self.objectx, self.objecty, self.objectz] = [pnt[0], pnt[1], pnt[2]]
		except Exception as e:
			print(e)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
			return
		# (rows,cols,channels) = cv_image.shape
		# if cols > 60 and rows > 60 :
		# 	cv2.circle(cv_image, (50,50), 10, 255)

		try:
			blured = cv2.blur(cv_image, (5, 5))
			hsv = cv2.cvtColor(blured, cv2.COLOR_BGR2HSV)

			lower = np.array(self.hsvColor[self.color][0])
			upper = np.array(self.hsvColor[self.color][1])
			mask = cv2.inRange(hsv, lower, upper)
			tmp, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			cv2.drawContours(cv_image, contours, -1, (0, 0, 255), 1)

			maxArea = 0
			contIdx = -1
			for idx, c in enumerate(contours):
				rect = cv2.minAreaRect(c) 
				# return (minimum enclosing rectangle = [[centerx, centery], [width, height], rotate])
				if rect[1][0]*rect[1][1] > maxArea:
					maxArea = rect[1][0]*rect[1][1]
					contIdx = idx

			if contIdx == -1:
				return
			biggestRect = cv2.minAreaRect(contours[contIdx])
			self.centerx, self.centery = biggestRect[0]
			box = cv2.boxPoints(biggestRect)
			box = np.int0(box)
			cv2.drawContours(cv_image, [box], 0, (0, 0, 255))

			x0 = int(box[0, 0])
			y0 = int(box[0, 1])
			x1 = int(box[1, 0])
			y1 = int(box[1, 1])
			x2 = int(box[2, 0])
			y2 = int(box[2, 1])
			x3 = int(box[3, 0])
			y3 = int(box[3, 1])
			cor0 = ("{}:({}, {})".format(0, x0, y0))
			cor1 = ("{}:({}, {})".format(1, x1, y1))
			cor2 = ("{}:({}, {})".format(2, x2, y2))
			cor3 = ("{}:({}, {})".format(3, x3, y3))
			# cv2.putText(cv_image, cor0, (x0, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
			# cv2.putText(cv_image, cor1, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
			# cv2.putText(cv_image, cor2, (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
			# cv2.putText(cv_image, cor3, (x3, y3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)


			cv2.namedWindow('Kinect Image',cv2.WINDOW_AUTOSIZE)
			cv2.imshow("Kinect Image", cv_image)
			cv2.waitKey(3)
		except Exception as e:
			print(e)
#		try:
#			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#		except CvBridgeError as e:
#			print(e)

def move_to_bottle_and_grasp(grasper, bx, by, bz):
	print "move to bottle and grasp..."
	grasper.release()
	grasper.move_to_position(bx - 0.08, by - 0.14, bz + 0.3)
	grasper.move_to_position(bx - 0.08, by - 0.14, bz)
	grasper.move_to_position(bx, by, bz)
	grasper.grasp()
	grasper.move_to_position(bx, by, bz + 0.3)


def move_to_cup_and_pour_water(grasper, cx, cy, cz):
	print "move to cup and pour water..."
	grasper.move_to_position(cx + 0.087, cy - 0.05, cz + 0.15)
	grasper.rotate(-1.75)
	grasper.rotate(1.75)
	grasper.move_to_position(cx + 0.087, cy - 0.05, cz + 0.25)
	
		
def put_back_bottle(grasper, bx, by, bz):
	print "put back bottle..."
	grasper.move_to_position(bx, by, bz + 0.3)
	grasper.move_to_position(bx, by, bz)
	grasper.release()
	grasper.move_to_position(bx - 0.08, by - 0.14, bz)


def move_to_cup_and_grasp(grasper, cx, cy, cz):
	# calibrate
	print "move to cup and grasp..."
	grasper.move_to_position(cx - 0.08, cy - 0.14, cz + 0.3)
	grasper.move_to_position(cx - 0.08, cy - 0.14, cz)
	grasper.move_to_position(cx, cy, cz )
	grasper.grasp()
	grasper.move_to_position(cx, cy, cz + 0.3)
	grasper.move_to_position(cx + 0.23*(math.cos(math.pi/6) - 0.5), 
		cy - 0.23*(math.cos(math.pi/6) - 0.5), cz + 0.3, 0.6829, 0.1830, 0.6829, -0.1830)
	

def move_to_target_and_put_down(grasper, tx, ty, tz):
	print "move to target and put down..."
	grasper.move_to_position(tx, ty, tz + 0.3)
	grasper.move_to_position(tx, ty, tz)
	grasper.release()
	grasper.move_to_position(tx, ty, tz + 0.3)
	grasper.move_to_neutral()


def main(args):
	rospy.init_node('cup_detector' + sys.argv[1])

	# cd = CupDetector(sys.argv[1])
	# while not rospy.is_shutdown():
	# 	print(cd.objectx, cd.objecty, cd.objectx)
	# 	rospy.sleep(1)

	try:
		bottle_cd = CupDetector(sys.argv[1])
		cup_cd = CupDetector('yellow')
		grasper = Grasper(verbose = False)

		# use 5 seconds to detect the position of cup and bottle
		for i in range(3):
			print (bottle_cd.objectx, bottle_cd.objecty, bottle_cd.objectz)
			print (cup_cd.objectx, cup_cd.objecty, cup_cd.objectz)
			print()
			# if not bottle_cd.objectx == 0 and not cup_cd.objectx == 0:
			# 	break
			rospy.sleep(1)

		# hc = HandCamera('pink')
		
		bx, by, bz = bottle_cd.objectx, bottle_cd.objecty, bottle_cd.objectz
		bx += 0.05
		bz += 0.03

		cx, cy, cz = cup_cd.objectx, cup_cd.objecty, cup_cd.objectz
		cx += 0.045
		cy += 0.02598
		cz += 0.02

		move_to_bottle_and_grasp(grasper, bx, by, bz)		
		
		move_to_cup_and_pour_water(grasper, cx, cy, cz)
		
		put_back_bottle(grasper, bx, by, bz)
		
		move_to_cup_and_grasp(grasper, cx, cy, cz)
	except Exception, e:
		logerr(e)


	# try:
	# 	rospy.spin()
	# except KeyboardInterrupt:
	# 	print("Shutting down")
	cv2.destroyAllWindows()
 
if __name__ == '__main__':
	main(sys.argv)
