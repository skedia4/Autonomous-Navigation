import rospy
from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd

pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=10)

class Node():
	def __init__(self):
		self.hertz = 10
		self.state = 0

	def run(self):
		sleep_time = 0.00001
		if self.state % 4 == 0 or self.state % 4 == 0:
			sleep_time = 0.5

		self.rate = rospy.Rate(self.hertz)

		while not rospy.is_shutdown():
			left_msg = PacmodCmd()
			left_msg.header = Header()
			left_msg.ui16_cmd = 2

			off_msg = PacmodCmd()
			off_msg.header = Header()
			off_msg.ui16_cmd = 1

			right_msg = PacmodCmd()
			right_msg.header = Header()
			right_msg.ui16_cmd = 0

			if self.state % 4 == 0:
				pub.publish(left_msg)
			elif self.state % 4 == 2:
				pub.publish(right_msg)
			else:
				pub.publish(off_msg)
			
			rospy.sleep(sleep_time)

			# pub.publish(off_msg)
			# pub.publish(off_msg)
			# pub.publish(right_msg)
			# pub.publish(off_msg)
			self.state += 1
			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('sos_node', anonymous=True)
	node = Node()
	node.run()