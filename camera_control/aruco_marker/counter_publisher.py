import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
class SimplePublisher(Node):
	def __init__(self):
# call super() in the constructor in order to initialize the Node object with node name as only parameter
		super().__init__('counter_publisher')
		self.publisher_ = self.create_publisher(Int32, '/counter', 1)
		self.count = Int32()
		self.count.data = 0
		timer_period = 1.0 # define the timer period
		self.timer = self.create_timer(timer_period, self.talker_callback)
	def talker_callback(self):
		self.count.data+=1
		self.publisher_.publish(self.count)
def main(args=None):
	rclpy.init(args=args) # initialize the ROS communication
	simple_publisher = SimplePublisher() # declare the node constructor
	rclpy.spin(simple_publisher) # pause the program execution, waits for a request to kill the node (ctrl+c)
	simple_publisher.destroy_node() # Explicitly destroy the node
	rclpy.shutdown() # shutdown the ROS communication
if __name__ == '__main__':
	main()
