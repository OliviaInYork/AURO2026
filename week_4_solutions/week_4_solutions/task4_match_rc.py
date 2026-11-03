#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped

from enum import Enum

class State(Enum):
    FORWARD = 0
    TURNING = 1

class Task2(Node):

    def __init__(self):
        super().__init__('task2')

        self.state = State.FORWARD
        self.counter = 0

        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.entered = False

    def control_loop(self):

        # Unlike the model answer in 'task4_match.py', in this solution we keep
        # track of whether the state has been entered via the variable 'entered', 
        # so that cmd_vel is only published once per state, ie. in its entry action.

        match self.state:
            case State.FORWARD:
                if not self.entered:
                    msg = TwistStamped()
                    msg.twist.linear.x = 0.3
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Forward: {msg}")
                    self.entered = True
                
                if self.counter > 4 * (1 / self.timer_period):
                    self.entered = False
                    self.state = State.TURNING
                    self.counter = 0
                self.counter += 1
            case State.TURNING:
                if not self.entered:
                    msg = TwistStamped()
                    msg.twist.angular.z = 1.5
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Turning: {msg}")
                    self.entered = True

                if self.counter > 1 * (1 / self.timer_period):
                    self.entered = False
                    self.state = State.FORWARD
                    self.counter = 0
                self.counter += 1
            case _:
                pass

def main(args=None):
    rclpy.init(args=args)

    turtlebot3_fsm = Task2()

    rclpy.spin(turtlebot3_fsm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtlebot3_fsm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()