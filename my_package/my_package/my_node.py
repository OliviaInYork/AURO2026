import rclpy
import signal

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped

class my_node(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, 'topic', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.guard_condition = self.create_guard_condition(self.shutdown_hook)
        self.old_handler = signal.signal(signal.SIGINT, self.sigint_handler)

    def shutdown_hook(self):
        self.get_logger().info('Shutting down node...')
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f"Stopping robot by publishing: '{msg}'")
        rclpy.shutdown()

    def sigint_handler(self, signum, frame):
        self.guard_condition.trigger()
        signal.signal(signal.SIGINT, self.old_handler)

    def timer_callback(self):
        msg = TwistStamped()
        msg.twist = Twist()
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
