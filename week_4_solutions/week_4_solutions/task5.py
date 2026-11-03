#!/usr/bin/env python3

import signal
from typing import Callable

from geometry_msgs.msg import Twist, TwistStamped, Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from week_4.robochart_fsm import Clock, Const, Required, Scope, State, StateMachine

lvel = 0.1  # Linear velocity
avel = 0.5  # Angular velocity


class TurtleBot3FSMC(StateMachine):

    # Constants, given at construction, and clocks
    F: Const[int]
    T: Const[int]
    clock_C: Clock

    # Provided by the robotic platform: requires IOperations
    cmd_vel: Required[Callable[[TwistStamped], None]]

    def initial(self):
        return self.Forward

    class Forward(State):

        def entry(self):
            self.cmd_vel(TwistStamped(header=Header(),
                                      twist=Twist(linear=Vector3(x=lvel, y=0.0, z=0.0),
                                                  angular=Vector3(x=0.0, y=0.0, z=0.0))))
            self.logger.info("In state 'FORWARD'")

        # Example of using a clock to trigger a transition after T time units
        def transitions(self):
            if self.clock_C.since() >= self.F:
                self.logger.info("Transition from 'FORWARD' to 'TURNING")
                # Target state, and the transition's action
                return self.Turning, self.clock_C.reset

    class Turning(State):

        def entry(self):
            self.cmd_vel(TwistStamped(header=Header(),
                                      twist=Twist(linear=Vector3(x=0.0, y=0.0, z=0.0),
                                                  angular=Vector3(x=0.0, y=0.0, z=avel))))
            self.logger.info("In state 'TURNING'")

        # Check transitions out of state Turning. As in RoboChart, since(C) counts whole time
        # units, so since(C) > F is enabled after F+1 time units.
        def transitions(self):
            if self.clock_C.since() > self.T:
                self.logger.info("Transition from 'TURNING' to 'FORWARD")
                return self.Forward, self.clock_C.reset


# The robotic platform, providing the cmd_vel operation: provides IOperations
class TurtleBot3Platform(Scope):
    cmd_vel: Callable[[TwistStamped], None]


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

        # Setup of robotic platform
        self.platform = TurtleBot3Platform(cmd_vel=self.cmd_vel)

        # Setup the state machine itself, with its constants given by ROS parameters
        self.stm = TurtleBot3FSMC(self.get_logger(), self.timer_period, self.time_unit,
                                  parent=self.platform,
                                  F=int(self.declare_parameter('F', 1).value),
                                  T=int(self.declare_parameter('T', 4).value))

    def cmd_vel(self, msg):
        self.get_logger().info(
            f'Publishing cmd_vel: linear.x={msg.twist.linear.x}, angular.z={msg.twist.angular.z}')
        self.publisher.publish(msg)

    def control_loop(self):

        # We step the machine
        self.get_logger().debug('-- Executing control_loop. --')
        self.stm.execute()


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
