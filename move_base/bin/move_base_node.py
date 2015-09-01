#! /usr/bin/env python

import serial
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

#Initial commands for Pololu Servo Master
cmpct_target= 0x84
mini_target  =0xFF
cmpct_set_speed = 0x87
cmpct_set_accel = 0x89

#Target Position at Pololu Servo Master for each motor
m1_ = 0x01	#Front Right
m2_ = 0x00	#Front Left
m3_ = 0x02	#Rear Left
m4_ = 0x03	#Rear Right

pi=3.1416

class Motor_Control:
	def __init__(self):

		self.m1_target = 127
		self.m2_target = 127
		self.m3_target = 127
		self.m4_target = 127

		self.m1_vel = 0
		self.m2_vel = 0
		self.m3_vel = 0
		self.m4_vel = 0

		self.a_ = rospy.get_param( 'a', .375/2 ) #componente x de la distancia entre centro de la base y el punto de contacto de la llanta 1
		self.b_ = rospy.get_param( 'b', .350/2 ) #componente y de la distancia entre centro de la base y el punto de contacto de la llanta 1

		self.serial = serial.Serial( "/dev/ttyACM0", 9600 )
		self.serial.open()

		self.serial.write( serial.to_bytes( [cmpct_target, m1_, 127] ) )
		self.serial.write( serial.to_bytes( [cmpct_target, m2_, 127] ) )
		self.serial.write( serial.to_bytes( [cmpct_target, m3_, 127] ) )
		self.serial.write( serial.to_bytes( [cmpct_target, m4_, 127] ) )

		self.serial.write( serial.to_bytes( [cmpct_set_accel, m1_, 0x00, 0x05] ) )
		self.serial.write( serial.to_bytes( [cmpct_set_accel, m2_, 0x00, 0x05] ) )
		self.serial.write( serial.to_bytes( [cmpct_set_accel, m3_, 0x00, 0x05] ) )
		self.serial.write( serial.to_bytes( [cmpct_set_accel, m4_, 0x00, 0x05] ) )

		self.serial.write( serial.to_bytes( [cmpct_set_speed, m1_, 0x00, 0x05] ) )
		self.serial.write( serial.to_bytes( [cmpct_set_speed, m2_, 0x00, 0x05] ) )
		self.serial.write( serial.to_bytes( [cmpct_set_speed, m3_, 0x00, 0x05] ) )
		self.serial.write( serial.to_bytes( [cmpct_set_speed, m4_, 0x00, 0x05] ) )

		
		
		rospy.init_node('move_base', anonymous=False)
		self.GET_VEL = rospy.Subscriber('cmd_vel', Twist, self.velCallback)
		self.GET_ACCEL = rospy.Subscriber('sensor_accel', Vector3, self.accelCallback)
		self.GET_ORIENT = rospy.Subscriber('sensor_orient', Vector3, self.orientCallback)
		rospy.spin()

	def velCallback(self,Twist):

		x_vel = Twist.linear.x
		y_vel = Twist.linear.y
		w_vel = Twist.angular.z

		m_unit=[]
		m_unit.append(y_vel - x_vel + pi * w_vel * (self.a_ + self.b_))
		m_unit.append(y_vel + x_vel - pi * w_vel * (-self.a_ + self.b_))
		m_unit.append(y_vel - x_vel - pi * w_vel * (self.a_ + self.b_))
		m_unit.append(y_vel + x_vel + pi * w_vel * (self.a_ + self.b_))
		
		# m_unit.append( x_vel - y_vel + (4 / (3*pi)) * w_vel * (self.a_ + self.b_) ** 1/2)
		# m_unit.append( x_vel + y_vel + (4 / pi) * w_vel * (self.a_ + self.b_) ** 1/2)
		# m_unit.append( x_vel - y_vel - (4 / pi) * w_vel * (self.a_ + self.b_) ** 1/2)
		# m_unit.append( x_vel + y_vel - (4 / (3*pi)) * w_vel * (self.a_ + self.b_) ** 1/2)

		self.m1_target = int( 127 + 60 * ( m_unit[0] ) )
		self.m2_target = int( 127 + 60 * ( m_unit[1] ) )
		self.m3_target = int( 127 + 60 * ( m_unit[2] ) )
		self.m4_target = int( 127 + 60 * ( m_unit[3] ) )

		self.serial.write( serial.to_bytes( [mini_target, m1_, self.m1_target] ) )
		self.serial.write( serial.to_bytes( [mini_target, m2_, self.m2_target] ) )
		self.serial.write( serial.to_bytes( [mini_target, m3_, self.m3_target] ) )
		self.serial.write( serial.to_bytes( [mini_target, m4_, self.m4_target] ) )
			
	def accelCallback(self,Vector3):
		print "This will be something"
	
	def orientCallback(self,Vector3):
		print "This will be something"

if __name__ == '__main__':
	Motor_Control()