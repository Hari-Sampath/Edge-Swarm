"""Thin Robot Operating System 2 (ROS 2) simulation clock observer."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class MetricsNode(Node):
    """Keep an independent count of ticks observed by ROS subscribers."""
    def __init__(self):
        super().__init__('metrics_observer'); self.last_tick=0; self.create_subscription(Int64,'/tick',self.observe,10)
    def observe(self,msg): self.last_tick=msg.data

def main(args=None):
    rclpy.init(args=args); n=MetricsNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
