#!/usr/bin/env python3

# Incomplete sketch for Week4's task 7.
#
# This Python program implements the TurtleBot3FSMC.

from enum import Enum
import math
import signal
from typing import Callable

from auro_interfaces.msg import Obstacle
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
from week_4.robochart_fsm import (Clock, FINAL, Input, Operation, Param, Required, Scope, State,
                                  StateMachine)

lvel = 0.3  # Linear velocity
avel = 0.1  # Angular velocity

# Operation move(lvel: real, avel: real), defined by a state machine.
class Move(Operation):

    lvel: Param[float]
    avel: Param[float]

    # Provided by the caller: requires IOperations
    cmd_vel: Required[Callable[[TwistStamped], None]]

    # Initial junction leads to final state
    def initial(self):
        # TODO: Implement call to self.cmd_vel, passing self.lvel, self.avel.
        return FINAL

class TurtleBot3FSMC(StateMachine):

    # Constants, given at construction, and clocks
    F: Const[int]
    T: Const[int]
    clock_C: Clock

    # Provided by the robotic platform: requires IOperations
    cmd_vel: Required[Callable[[TwistStamped], None]]
    move: Required[Operation]

    def initial(self):
        return self.Forward

    class Forward(State):

        def entry(self):
            yield from self.move(lvel, 0)
            self.logger.info("In state 'FORWARD'")

        # Check transitions out of state Forward.
        def transitions(self):
            # TODO: Implement transitions

    class Turning(State):

        def entry(self):
            yield from self.move(0, avel)
            self.logger.info("In state 'TURNING'")

        # Check transitions out of state Turning.
        def transitions(self):
            # TODO: Implement transitions    

# The robotic platform: its operations and events.
class TurtleBot3Platform(Scope):
    cmd_vel: Callable[[TwistStamped], None]
    
# The controller: it provides the operations used by the state machine.
class TurtleBot3Controller(Scope):

    # Here, we define that the operation move is defined by the implementation
    # in the class Move above.
    move: type[Operation] = Move

class TurtleBot3FSMCNode(Node):

    def __init__(self):
        super().__init__('TurtleBot3FSMC')

        # ROS2 Outputs
        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 1)

        # Periodic callback for executing FSM
        self.timer_period = 0.1  # 100 milliseconds = 10 Hz for the control loop
        # 1 second is the value chosen for one 'time unit': a whole number of timer periods
        self.time_unit = 1

        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Setup of robotic platform, providing the cmd_vel operation, and controller, providing
        # the implementation of move.
        self.platform = TurtleBot3Platform(cmd_vel=self.cmd_vel)
        self.controller = TurtleBot3Controller(parent=self.platform)

        # Setup the state machine itself. This checks that everything it requires is provided.
        self.stm = SampleTurtleBot3FSMC(self.get_logger(), self.timer_period, self.time_unit,
                                        parent=self.controller)

    def cmd_vel(self, msg):
        self.get_logger().info(
            f'Publishing cmd_vel: linear.x={msg.twist.linear.x}, angular.z={msg.twist.angular.z}')
        self.publisher.publish(msg)

    def control_loop(self):

        # We step the machine
        self.get_logger().debug('-- Executing control_loop. --')
        self.stm.execute()
        self.get_logger().debug(f'-- End control_loop: {self.stm.configuration()} --')

def main(args=None):

    # On Ctrl+C (SIGINT), rclpy's own handler shuts down ROS, which stops spin below. Python
    # would also raise KeyboardInterrupt, wherever the program happens to be, so we disable that.
    signal.signal(signal.SIGINT, lambda signum, frame: None)

    rclpy.init(args=args)

    node = TurtleBot3FSMCNode()

    # Stop cleanly on Ctrl+C (SIGINT) or a request to terminate (SIGTERM).
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # On SIGTERM, rclpy shuts down its context, which can interrupt the executor with an
        # error: only report errors that are not due to the shutdown.
        if rclpy.ok():
            raise
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
