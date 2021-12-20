#!/usr/bin/env python2.7
import numpy as np
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from mavros_msgs.msg import AttitudeTarget, PositionTarget
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist, TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from constrained_time_opt_new import min_snap
from math import factorial as f
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


class FLIGHT_CONTROLLER:

	def __init__(self):
		self.pt = Point()

		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS																	
		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		# self.get_linear_vel=rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel,)
		self.get_imu_data=rospy.Subscriber('/mavros/imu/data',Imu,self.get_euler_angles)

		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
		self.publish_pos_tar = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=0)

		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

		rospy.loginfo('INIT')

	def toggle_arm(self, arm_bool):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			self.arm_service(arm_bool)
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)

	def takeoff(self, t_alt):
		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			self.takeoff_service(0,0,0,0,t_alt)
			rospy.loginfo('TAKEOFF')
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)
	
	
	def land(self, l_alt):
		rospy.wait_for_service('/mavros/cmd/land')
		try:
			self.land_service(0.0, 0.0, 0, 0, l_alt)
			rospy.loginfo("LANDING")
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)


	def set_mode(self,md):
			rospy.wait_for_service('/mavros/set_mode')
			try:
				self.flight_mode_service(0, md)
				rospy.loginfo("Mode changed")	
			except rospy.ServiceException as e:
				rospy.loginfo("Mode could not be set: " %e)

	def set_Guided_mode(self):
		rate=rospy.Rate(20)
		PS = PoseStamped()
		PS.pose.position.x = self.pt.x
		PS.pose.position.y = self.pt.y
		PS.pose.position.z = self.pt.z
		for i in range(10):
			self.publish_pose.publish(PS)	
			rate.sleep()
		print('done')
		self.set_mode("GUIDED")

	def get_pose(self, location_data):
		self.pt.x = location_data.pose.position.x
		self.pt.y = location_data.pose.position.y
		self.pt.z = location_data.pose.position.z

	def get_euler_angles(self,orientaion_data):
		orientation_q = orientaion_data.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_list)


	#PUBLISHERS
	def gotopose(self,x,y,z):
		rate = rospy.Rate(20)
		sp = PoseStamped()
		sp.pose.position.x = x
		sp.pose.position.y = y
		sp.pose.position.z = z
		sp.pose.orientation.x = 0.0
		sp.pose.orientation.y = 0.0
		sp.pose.orientation.z = 0.0
		sp.pose.orientation.w = 1.0
		dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
		while(dist > 0.2):
			self.publish_pose.publish(sp)
			dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
			rate.sleep()
		#print('Reached ',x,y,z)


	def set_pos(self, a_x, a_y, a_z, v_x, v_z, v_y, x, y, z):
		sp = PositionTarget()
		sp.coordinate_frame = 1
		sp.type_mask = 3072				
		sp.acceleration_or_force.x = a_x
		sp.acceleration_or_force.y = a_y
		sp.acceleration_or_force.z = a_z
		sp.velocity.x = v_x
		sp.velocity.y = v_y
		sp.velocity.z = v_z
		sp.position.x = x
		sp.position.y = y
		sp.position.z = z
		self.publish_pos_tar.publish(sp)



if __name__ == '__main__':

	
	mav = FLIGHT_CONTROLLER()
	rate = rospy.Rate(10)

	x = [0,5,5,0,0]
	y = [0,0,5,5,0]
	z = [5,5,5,5,5]
	v_test = 2
	v_min=0.1
	v_max=15
	ms = min_snap(x,y,z,v_test,v_min,v_max)
	ms.optimize()
	ms.get_trajectory_var()

	mav.toggle_arm(1)
	time.sleep(1)
	mav.set_Guided_mode()
	mav.takeoff(5)
	time.sleep(10)
	print("Starting")
	i = 0
	while (True):
		print("Traj")
		a_x = ms.x_dot_dot_path[i]
		a_y = ms.y_dot_dot_path[i]
		a_z = ms.z_dot_dot_path[i]
		v_x = ms.x_dot_path[i]
		v_y = ms.y_dot_path[i]
		v_z = ms.z_dot_path[i]
		x = ms.x_path[i]
		y = ms.y_path[i]
		z = ms.z_path[i]
		i = i+1
		mav.set_pos(a_x, a_y, a_z, v_x, v_y, v_z, x, y, z)
		rate.sleep()
	
	plt.figure(figsize=(10,5))
	ax = plt.axes(projection ='3d')
	ms.plot('g','Time Optimized Trajectory')

