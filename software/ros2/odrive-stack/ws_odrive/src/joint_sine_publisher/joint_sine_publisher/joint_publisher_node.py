import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.declare_parameter('amplitude', 0.5)
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('topic', '/position_controller/commands')

        self.amp = self.get_parameter('amplitude').value
        self.freq = self.get_parameter('frequency').value
        self.topic = self.get_parameter('topic').value

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.start_time = time.time()
        self.timer = self.create_timer(0.02, self.publish_cmd)

    def publish_cmd(self):
        t = time.time() - self.start_time
        value = self.amp * math.sin(2 * math.pi * self.freq * t)
        msg = Float64MultiArray()
        msg.data = [value]
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main():
    rclpy.init()
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
