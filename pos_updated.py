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
		# self.vel = Point()
		# self.acc = Point()

		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS																	
		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		self.get_linear_vel=rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel)
		self.get_imu_data=rospy.Subscriber('/mavros/imu/data',Imu,self.get_euler_angles)
		self.get_imu_data_acc=rospy.Subscriber('/mavros/imu/data',Imu,self.get_acc)

		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
		self.publish_pos_tar = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=1)

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

	# def get_vel(self, location_data):
	# 	self.vel.x = location_data.twist.linear.x
	# 	self.vel.y = location_data.twist.linear.y
	# 	self.vel.z = location_data.twist.linear.z

	def get_vel(self, vel_data):
		vel = vel_data.twist.linear
		self.vel = np.array([vel.x, vel.y, vel.z]).transpose()

	def get_acc(self, imu_data_acc):
		acc = imu_data_acc.linear_acceleration
		self.acc = np.array([acc.x, acc.y, acc.z]).transpose()

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


	def set_pos(self, a):
		sp = PositionTarget()
		sp.coordinate_frame = 1
		sp.type_mask = 3135
		sp.acceleration_or_force.x = a[0]
		sp.acceleration_or_force.y = a[1]
		sp.acceleration_or_force.z = -a[2]
		# sp.velocity.x = v_x
		# sp.velocity.y = v_y
		# sp.velocity.z = v_z
		# sp.position.x = p_x
		# sp.position.y = p_y
		# sp.position.z = p_z
		self.publish_pos_tar.publish(sp)


class smc:
	
	def __init__(self,m,alpha_1,alpha_2,beta,d_p,dt):
		# Gravity Vector
		self.g = np.array([0 , 0 , -9.8]).transpose()
		self.m = m
		self.alpha_1 = alpha_1
		self.alpha_2 = alpha_2
		self.beta=beta
		self.d_p = d_p
		self.dt = dt

	def controller(self, p, p_dot, p_ddot, p_d, p_dot_d, p_ddot_d, psi_d, p_e, v_e, s_0, t):
		# Numerical Integration
		p_e_u = p_e + v_e*self.dt
		v_e_u = v_e + (p_ddot_d - p_ddot)*self.dt
		s_0_u = s_0 + (self.alpha_1*p_e + self.alpha_2*v_e)*self.dt + (v_e_u - v_e)

		self.p_e_u = p_e_u
		self.v_e_u = v_e_u
		self.s_0_u = s_0_u
			
		E = self.alpha_1*p_e_u + self.alpha_2*v_e_u + p_ddot_d + np.dot(self.d_p,p_dot)/self.m
		E_hat = E + 0*self.beta[1]*np.sign(s_0_u)
			
		phi_d = np.arcsin((E_hat[0]*np.sin(psi_d) - E_hat[1]*np.cos(psi_d))/np.linalg.norm(E_hat, 2))
		theta_d = np.arctan((E_hat[0]*np.cos(psi_d) + E_hat[1]*np.sin(psi_d))/E_hat[2])
		u_T = np.dot((np.array([np.cos(phi_d)*np.cos(psi_d)*np.sin(theta_d) + np.sin(psi_d)*np.sin(phi_d), np.sin(theta_d)*np.sin(psi_d)*np.cos(phi_d) - np.cos(psi_d)*np.sin(phi_d), np.cos(phi_d)*np.cos(theta_d)])).transpose(), E_hat)
		
		self.p_ddot_c = E_hat + self.g - np.dot(self.d_p,p_dot)/self.m + 0*np.array([np.sin(t), np.sin(t), np.sin(t)]).transpose()


def actuate(x, y, z, v_test, v_min, v_max):

	mav = FLIGHT_CONTROLLER()
	mav.toggle_arm(1)
	time.sleep(1)
	mav.set_Guided_mode()
	mav.takeoff(5)
	time.sleep(5)
	
	m = 0.47												# mass of the quadrotor
	alpha_1 = 10
	alpha_2 = 2
	beta = np.array([0.01,1.5,2,7])
	J_p = np.diag(np.array([4.9e-3, 4.9e-3, 8.8e-3]))		# coefficients of the rotary inertia
	Tau_n = np.array([1, 1, 1]).transpose()					# moments in the body-fixed frame
	d_p = np.diag(np.array([0.00,0.00,0.00]))				# air drag
	d_eta = np.diag(np.array([6e-5,6e-5,6e-5]))				# aerodynamic drag coefficients

	ms = min_snap(x, y, z, v_test, v_min, v_max)
	ms.optimize()
	x_path,x_dot_path,x_ddot_path,y_path,y_dot_path,y_ddot_path,z_path,z_dot_path,z_ddot_path,psi_path = ms.get_trajectory_var()

	Smc = smc(m, alpha_1, alpha_2, beta, d_p, dt=0.2)

	p = np.array([0, 0, 10]).transpose()
	p_dot = np.array([0, 0, 0]).transpose()
	p_ddot = np.array([0, 0, 0]).transpose()
	psi = 0

	x_true = [p[0]]
	y_true = [p[1]]
	z_true = [p[2]]
	psi_true = [psi]

	time_arr = [0]
	r_err = [np.sqrt((x_path[0]-x_true[0])**2 +(y_path[0]-y_true[0])**2 + (z_path[0]-z_true[0])**2)]

	p_e = np.zeros((3,len(z_path)+1))
	v_e = np.zeros((3,len(z_path)+1))
	s_0 = np.zeros((3,len(z_path)+1))
		
	p_e[:,0] = np.array([x_path[0], y_path[0], z_path[0]]).transpose() - p
	v_e[:,0] = np.array([x_dot_path[0], y_dot_path[0], z_dot_path[0]]).transpose() - p_dot
	s_0[:,0] = alpha_2*p_e[:,0] + v_e[:,0]

	rate = rospy.Rate(5)
	irate = rospy.Rate(20)

	for iter in range(len(z_path)):

		p_d = np.array([x_path[iter], y_path[iter], z_path[iter]]).transpose()
		p_dot_d = np.array([x_dot_path[iter], y_dot_path[iter], z_dot_path[iter]]).transpose()
		p_ddot_d = np.array([x_ddot_path[iter], y_ddot_path[iter], z_ddot_path[iter]]).transpose()
		psi_d = np.arctan2(p_dot_d[1],p_dot_d[0])

		Smc.controller(p, p_dot, p_ddot, p_d, p_dot_d, p_ddot_d, psi_d, p_e[:,iter], v_e[:,iter], s_0[:,iter], iter*0.2)
		
		for i in range (4):
			mav.set_pos(Smc.p_ddot_c)
			irate.sleep()
		
		rate.sleep()

		p = np.array([mav.pt.x, mav.pt.y, mav.pt.z]).transpose()
		p_dot = mav.vel
		p_ddot = mav.acc

		x_true.append(p[0])
		y_true.append(p[1])
		z_true.append(p[2])
		print("Commanded:",Smc.p_ddot_c)
		print("Actual: ",p_ddot)

		r_err.append(np.sqrt((x_path[iter]-p[0])**2 + (y_path[iter]-p[1])**2 + (z_path[iter]-p[2])**2))
		time_arr.append(time_arr[-1]+0.01)

		p_e[:,iter+1] = np.copy(Smc.p_e_u)
		v_e[:,iter+1] = np.copy(Smc.v_e_u)
		s_0[:,iter+1] = np.copy(Smc.s_0_u)

	
	fig = plt.figure()
	ax = plt.axes(projection = '3d')
	ax.plot3D(x_path, y_path, z_path, linestyle = '-', marker = '.', color = 'red')
	ax.plot3D(x_true, y_true, z_true, linestyle = '-', color = 'blue')

	ax.set_title('Flight path').set_fontsize(20)
	ax.set_xlabel('$x$').set_fontsize(20)
	ax.set_ylabel('$y$').set_fontsize(20)
	ax.set_zlabel('$z$').set_fontsize(20)
	plt.legend(['Planned path','Executed path'], fontsize = 14)

	plt.show()

	plt.plot(time_arr, r_err)
	plt.show()

if __name__ == '__main__':

	x=[0,0,0,0,0]
	y=[0,0,0,0,0]
	z=[5,2,8,3,7]

	v_test = 0.5
	v_min=0.05
	v_max=1

	actuate(x,y,z,v_test,v_min,v_max)
