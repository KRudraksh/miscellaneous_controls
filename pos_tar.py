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
		self.vel = Point()

		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS																	
		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		self.get_linear_vel=rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel)
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

	def get_vel(self, location_data):
		self.vel.x = location_data.twist.linear.x
		self.vel.y = location_data.twist.linear.y
		self.vel.z = location_data.twist.linear.z

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


	def set_pos(self, a_x, a_y, a_z):
		sp = PositionTarget()
		sp.coordinate_frame = 1
		sp.type_mask = 3072				
		sp.acceleration_or_force.x = a_x
		sp.acceleration_or_force.y = a_y
		sp.acceleration_or_force.z = a_z
		# sp.velocity.x = v_x
		# sp.velocity.y = v_y
		# sp.velocity.z = v_z
		# sp.position.x = p_x
		# sp.position.y = p_y
		# sp.position.z = p_z
		self.publish_pos_tar.publish(sp)


if __name__ == '__main__':

	
	mav = FLIGHT_CONTROLLER()
	rate = rospy.Rate(10)
	rate_f = rospy.Rate(100)

	x = [0,5,5,0,0]
	y = [0,0,5,5,0]
	z = [5,5,5,5,5]
	v_test = 2
	v_min=0.1
	v_max=10
	kp_pos = np.array([19,19,20])
	kd_pos = np.array([14,14,15])
	ms = min_snap(x,y,z,v_test,v_min,v_max)
	ms.optimize()
	ms.get_trajectory_var()

	mav.toggle_arm(1)
	time.sleep(1)
	mav.set_Guided_mode()
	mav.takeoff(5)
	time.sleep(10)
	print("Starting")
	n = len(ms.x_path)

	#PID 
	x_actual = []
	y_actual = []
	z_actual = []
	for j in range(n):
		print(mav.vel.x)
		a_x = kp_pos[0]*(ms.x_path[j]-mav.pt.x) + kd_pos[0]*(ms.x_dot_path[j]-mav.vel.x) + ms.x_dot_dot_path[j]
		a_y = kp_pos[1]*(ms.y_path[j]-mav.pt.y) + kd_pos[1]*(ms.y_dot_path[j]-mav.vel.y) + ms.y_dot_dot_path[j]
		a_z = kp_pos[2]*(ms.z_path[j]-mav.pt.z) + kd_pos[2]*(ms.z_dot_path[j]-mav.vel.z) + ms.z_dot_dot_path[j]
		mav.set_pos(a_x, a_y, a_z)
		x_actual.append(mav.pt.x)
		y_actual.append(mav.pt.y)
		z_actual.append(mav.pt.z)
		rate.sleep()		
	
	for k in range(50):
		x_actual.append(mav.pt.x)
		y_actual.append(mav.pt.y)
		z_actual.append(mav.pt.z)
		rate.sleep()
		
	ax = plt.axes(projection ='3d')
	ax.scatter(ms.x, ms.y, ms.z, c='black',marker='o',s=20)

	for v in range(ms.m):
		w,u,a=[],[],[]
		r=np.linspace(ms.t[v],ms.t[v+1],100)
		for i in range(100):
			g,e,f=0,0,0
			for j in range(ms.n*v,(v+1)*ms.n):
				g=g+(ms.p_x[j]*pow(r[i],j-(ms.n*v)))
				e=e+(ms.p_y[j]*pow(r[i],j-(ms.n*v)))
				f=f+(ms.p_z[j]*pow(r[i],j-(ms.n*v)))
			w.append(g)
			u.append(e)
			a.append(f)
		ax.plot3D(w, u, a, 'r')
		   
		
	ax.plot3D(w, u, a, 'r' ,label='Time Optimized')
	print(n)	
	
	print(x_actual)
	ax.scatter(x_actual, y_actual, z_actual, c='green',marker='o',s=15, label='Actual Trajectory' )
	plt.legend()
	plt.show()
