#!/usr/bin/env python3

# Sample solution for Week4's task 4.
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


class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    FRONT = 2

    @classmethod
    def default(cls):
        return cls.LEFT  # Provide a default value


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

        # Example of using a clock to trigger a transition after F time units
        def transitions(self):
            if self.clock_C.since() >= self.F:
                self.logger.info("Transition from 'FORWARD' to 'TURNING")
                # Target state, and the transition's action
                return self.Turning, self.clock_C.reset

    class Turning(State):

        def entry(self):
            yield from self.move(0, avel)
            self.logger.info("In state 'TURNING'")

        # Check transitions out of state Turning. As in RoboChart, since(C) counts whole time
        # units, so since(C) > T is enabled after T+1 time units.
        def transitions(self):
            if self.clock_C.since() > self.T:
                self.logger.info("Transition from 'TURNING' to 'FORWARD")
                return self.Forward, self.clock_C.reset

# The robotic platform: its operations and events.
class TurtleBot3Platform(Scope):
    cmd_vel: Callable[[TwistStamped], None]
    yaw: Input[float]
    obstacle: Input[Direction]


# The controller: it provides the operations used by the state machine.
class TurtleBot3Controller(Scope):
    move: type[Operation] = Move
    rotate: type[Operation] = Rotate


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
        # move and rotate
        self.platform = TurtleBot3Platform(cmd_vel=self.cmd_vel)
        self.controller = TurtleBot3Controller(parent=self.platform)

        # Setup the state machine itself. This checks that everything it requires is provided.
        self.stm = SampleTurtleBot3FSMC(self.get_logger(), self.timer_period, self.time_unit,
                                        parent=self.controller)

    def cmd_vel(self, msg):
        self.get_logger().info(
            f'Publishing cmd_vel: linear.x={msg.twist.linear.x}, angular.z={msg.twist.angular.z}')
        self.publisher.publish(msg)

    def obstacle_detected(self, msg):
        self.get_logger().debug("Received message on topic 'obstacle'")

        # We set the event in the platform, passing the data, mapping from a ROS message
        # type, in this case, Obstacle, to a type that corresponds to that used by the
        # implementation of the RoboChart machine above in SampleTurtleBot3FSMC.
        #
        # Note that we could have used the same ROS type instead.
        match msg.data:
            case Obstacle.LEFT:
                self.platform.obstacle.set(Direction.LEFT)
            case Obstacle.FRONT:
                self.platform.obstacle.set(Direction.FRONT)
            case Obstacle.RIGHT:
                self.platform.obstacle.set(Direction.RIGHT)

        # Step the machine in response to the event, without stepping its clocks. If not
        # consumed in this step, the event remains available till consumed, or replaced by a
        # newer one.
        self.stm.execute(timed=False)

    def odom_callback(self, msg):

        self.get_logger().debug("Received message on topic 'odom'")

        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]

        (_, _, yaw) = euler_from_quaternion(q)

        # Convert to RoboChart type 'real', here float in the Python implementation.
        self.platform.yaw.set(yaw)

        # Step the machine in response to the event, without stepping its clocks.
        self.stm.execute(timed=False)

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
