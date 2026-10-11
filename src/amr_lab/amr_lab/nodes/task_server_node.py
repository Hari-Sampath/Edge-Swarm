"""Thin Robot Operating System 2 (ROS 2) task status observer."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TaskServerNode(Node):
    """Log task allocation events emitted by simulation users."""
    def __init__(self):
        super().__init__('task_server'); self.create_subscription(String,'/tasks/status',self.status,10)
    def status(self,msg): self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args); n=TaskServerNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
