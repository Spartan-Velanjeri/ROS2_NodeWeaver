
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
    

class publisher_xm24pq5(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher('', '', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self): # replace this with your function which gives the content to publish
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    

def main(args=None):
    rclpy.init(args=args) #ROS
    

    publisher_xm24pq5_node = publisher_xm24pq5()
    rclpy.spin(publisher_xm24pq5_node) #ROS

    
    rclpy.shutdown() #ROS
if __name__ == '__main__':
    main()
    