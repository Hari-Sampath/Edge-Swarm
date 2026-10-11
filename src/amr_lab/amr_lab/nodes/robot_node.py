"""Thin Robot Operating System 2 (ROS 2) observer for a simulated bus."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64

class RobotBusNode(Node):
    """Observe the namespaced robot position and simulation ticks."""
    def __init__(self):
        super().__init__('robot_bus_observer'); self.declare_parameter('robot_id','robot_0')
        name=self.get_parameter('robot_id').value
        self.create_subscription(String,f'/bus/{name}',self.message,10); self.create_subscription(Int64,'/tick',self.tick,10)
    def message(self,msg): self.get_logger().debug(msg.data)
    def tick(self,msg): pass

def main(args=None):
    rclpy.init(args=args); n=RobotBusNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
