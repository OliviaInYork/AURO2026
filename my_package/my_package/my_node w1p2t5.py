import rclpy
import signal

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped

class my_node(Node):

    def __init__(self):
        super().__init__('Task5_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        msg.twist = Twist()
        msg.twist.angular.z = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{msg}'")


def main(args=None):
    rclpy.init(args=args)
    node = my_node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
