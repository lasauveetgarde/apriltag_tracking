#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math
import queue # FIFO queue
import time # Tracking time

class PIDController:
	def __init__(self, kP, kI, kD, kS):
		self.kP       = kP # Proportional gain
		self.kI       = kI # Integral gain
		self.kD       = kD # Derivative gain
		self.kS       = kS # Saturation constant (error history buffer size)
		self.err_int  = 0 # Error integral
		self.err_dif  = 0 # Error difference
		self.err_prev = 0 # Previous error
		self.err_hist = queue.Queue(self.kS) # Limited buffer of error history
		self.t_prev   = 0 # Previous time

	def control(self, err, t):
		dt = t - self.t_prev # Timestep
		if dt > 0.0:
			self.err_hist.put(err) # Update error history
			self.err_int += err # Integrate error
			if self.err_hist.full(): # Jacketing logic to prevent integral windup
				self.err_int -= self.err_hist.get() # Rolling FIFO buffer
			self.err_dif = (err - self.err_prev) # Error difference
			u = (self.kP * err) + (self.kI * self.err_int * dt) + (self.kD * self.err_dif / dt) # PID control law
			self.err_prev = err # Update previos error term
			self.t_prev = t # Update timestamp
			return u # Control signal

# Node class
class RobotController:

	def __init__(self):
		rospy.init_node("speed_controller")
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
		# timer_period = 0.001 # Node execution time period (seconds)
		# self.timer = self.create_timer(timer_period, self.robot_controller_callback) # Define timer to execute 'robot_controller_callback()' every 'timer_period' seconds
		self.ctrl_msg = Twist() # Robot control commands (twist)
		# self.start_time = self.get_clock().now() # Record current time in seconds
		self.pid_lon = PIDController(0.2, 0.001, 0.05, 10) # Longitudinal PID controller object initialized with kP, kI, kD, kS
		self.pid_lat = PIDController(2.5, 0.0, 0.0, 10) # Lateral PID controller object initialized with kP, kI, kD, kS
		self.sub = rospy.Subscriber("/odom", Odometry, self.robot_controller_callback)
		# Init variables: robot position, orientation
		self.need_x = -2.0
		self.need_y = 2.0
	def robot_controller_callback(self, msg):

		goal = Point()
		goal.x = self.need_x
		goal.y = self.need_y


		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		self.rot_q = msg.pose.pose.orientation
		_, _, self.theta = euler_from_quaternion([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])
		# if self.get_clock().now() - self.start_time > Duration(seconds=DELAY):
		# 	to_frame_rel = 'camera'
		# 	from_frame_rel = 'tag36h11_0'
		# 	try:
		# 		tf2_msg = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
		# 	except TransformException as e:
		# 		# self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}')
		# 		print('No AprilTag marker found, looking for one...')
		# 		return

			# lon_error = tf2_msg.transform.translation.z # Calculate longitudinal error w.r.t. AprilTag marker
			# lat_error = -tf2_msg.transform.translation.x # Calculate lateral error w.r.t. AprilTag marker
		lon_error = goal.x - self.x
		inc_x = goal.x - self.x
		inc_y = goal.y - self.y
		lat_error = math.atan2(inc_y,inc_x) - self.theta
		lon_error = math.sqrt(inc_x**2 + inc_y**2)
		print(f'current ang error {lat_error}')
		tstamp = time.time() # Current timestamp (s)
		if abs(lat_error) > 0.1 or math.sqrt(inc_x**2 + inc_y**2) > 0.1:
			# LIN_VEL = 0.2
			LIN_VEL = self.pid_lon.control(lon_error, tstamp) # Linear velocity (m/s)

			ANG_VEL = self.pid_lat.control(lat_error, tstamp) # Angular velocity (rad/s)
			self.ctrl_msg.linear.x = LIN_VEL # Set linear velocity
			self.ctrl_msg.angular.z = ANG_VEL # Set angular velocity
			print('Deviation from AprilTag marker {}, {}'.format(round(lon_error, 4), round(lat_error, 4)))
		else:
			self.ctrl_msg.linear.x = 0.0
			self.ctrl_msg.angular.z = 0.0
			print('Stop.')
		self.pub.publish(self.ctrl_msg)
		




def main(args=None):
	RobotController() # Create node
	rospy.spin()

if __name__ == "__main__":
	main()