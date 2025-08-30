#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('first_node')
        self.counter_ = 0
        #self.get_logger().info('My first node has been started!')
        self.create_timer(1.0, self.timer_callback)  # Timer that calls the callback every second
    def timer_callback(self):
        self.get_logger().info('Timer callback executed!'+str(self.counter_))
        self.counter_ += 1
def main(args=None):
    rclpy.init(args=args)
    # Create a node
    my_node = MyNode()
    # Spin the node to keep it active
    rclpy.spin(my_node) #Ctrl C will stop the node to execute next line
    rclpy.shutdown()

if __name__ == '__main__':
    main()
