#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math


class ROSInter:
	def __init__(self):
		# Start a ROS node
		rospy.init_node("speed_controller")

		# Subsribe for topic
		self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

		# Create publisher
		# Message type: geometry_msgs/Twist
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

		# Init variables: robot position, orientation
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0

	# Callback for new odometry data
	def odom_callback(self, msg):
		# Store position and orientation
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		rot_q = msg.pose.pose.orientation
		_, _, self.theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

	# Function for moving robot to a certain position
	def move_to(self, x, y):
		print('Move to: (%d, %d)' % (x, y))
		speed = Twist()

		r = rospy.Rate(20)
		goal = Point()
		goal.x = x
		goal.y = y


		inc_x = goal.x - self.x
		inc_y = goal.y - self.y
		angle_to_goal = math.atan2(inc_y, inc_x)

		while abs(angle_to_goal - self.theta) > 0.1 or math.sqrt(inc_x**2 + inc_y**2) > 0.1:
			inc_x = goal.x - self.x
			inc_y = goal.y - self.y
			angle_to_goal = math.atan2(inc_y, inc_x)

			print('Position: (%f, %f' % (self.x, self.y))
			print(f'angle to goal is {angle_to_goal} robot theta is {self.theta}')

			print(f'errors is {angle_to_goal - self.theta}')
			Kx = 0.1
			Ky = 0.2
			# speed.linear.x = Kx * inc_x
			speed.linear.x = 0.5

			speed.angular.z = Ky * (angle_to_goal - self.theta)


			# if not (self.theta > 3.13 or self.theta < - 3.13):
			# 	if angle_to_goal - self.theta > 0.2:
			# 		print('Turning left...')
			# 		speed.angular.z = -0.3
			# 		speed.linear.x = 0.0
			# 	elif angle_to_goal - self.theta < -0.2:
			# 		print('Turning right...')
			# 		speed.angular.z = 0.3
			# 		speed.linear.x = 0.0
			# elif math.sqrt(inc_x**2 + inc_y**2) > 0.2:
			# 	print('Straight ahead...')
			# 	speed.linear.x = 0.5
			# 	speed.angular.z = 0.0

			self.pub.publish(speed)
			r.sleep()

		print('Stop.')
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		self.pub.publish(speed)

		return True

# Create class instance
ros_inter = ROSInter()
x = -5
y = -5
print(ros_inter.move_to(x, y))

