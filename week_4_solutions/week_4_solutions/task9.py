#!/usr/bin/env python3

# Sample solution for Week4's task 9.
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
from week_4.robochart_fsm import (Clock, FINAL, Input, junction, Operation, Param, Required,
                                  Scope, State, StateMachine)

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
        self.cmd_vel(TwistStamped(header=Header(),
                                  twist=Twist(linear=Vector3(x=self.lvel, y=0.0, z=0.0),
                                              angular=Vector3(x=0.0, y=0.0, z=self.avel))))
        return FINAL


# Operation rotate(angle: real), defined by a state machine.
class Rotate(Operation):

    # Parameter of the RoboChart operation, angle of type real, here using floats.
    angle: Param[float]

    # Local variables, which are reset on every call
    init: float = 0.0
    current: float = 0.0

    # Provided by the caller
    yaw: Required[Input[float]]

    # Initial junction, whose transition, with action yaw?init, leads to s0.
    def initial(self):

        # The action waits for the input yaw, so it may take several cycles.
        def action():
            while not self.yaw.triggered():
                yield
            self.init = self.yaw.take()

        return self.s0, action

    class s0(State):

        def transitions(self):
            if self.yaw.triggered():

                # Implementation of RoboChart trigger: yaw?current
                self.current = self.yaw.take()

                # Junction: if the robot has rotated by angle (within a tolerance), the
                # operation terminates, otherwise it waits for the next yaw in s0.
                @junction
                def j0():
                    rotated = abs(((self.current - self.init) + math.pi) % (2 * math.pi) - math.pi)
                    if self.angle - rotated <= 0.05:
                        return FINAL
                    else:
                        return self.s0

                # Transition yaw?current to the junction
                return j0


# We define SampleTurtleBot3FSMC by inheriting from StateMachine
class SampleTurtleBot3FSMC(StateMachine):

    # Variables and clocks
    obs: Direction = Direction.default()
    C: Clock

    # Provided by the controller and platform: events, and the operations called and their
    # requirements
    obstacle: Required[Input[Direction]]
    yaw: Required[Input[float]]
    cmd_vel: Required[Callable[[TwistStamped], None]]
    move: Required[Operation]
    rotate: Required[Operation]

    def initial(self):
        return self.Driving

    # Composite state: it has an initial junction, and its substates are nested classes.
    class Driving(State):

        def initial(self):
            # Initial transition with action #C
            return self.Forward, self.C.reset

        class Forward(State):

            def entry(self):
                self.logger.info("In substate 'Forward'")

                # An operation defined by a state machine may, in general, take a number of
                # steps/time, so we delegate to it with 'yield from'.
                yield from self.move(lvel, 0.0)

            # As in RoboChart, since(C) counts whole time units, so since(C) > 5 is enabled
            # after 6 time units.
            def transitions(self):
                if self.C.since() > 5:
                    return self.Turning, self.C.reset

        class Turning(State):

            def entry(self):
                self.logger.info("In substate 'Turning'")
                yield from self.move(0.0, avel)

            def transitions(self):
                if self.C.since() > 1:
                    return self.Forward, self.C.reset

        def entry(self):
            self.logger.info("In state 'Driving'")

        def transitions(self):
            if self.obstacle.triggered():

                # Implementation of RoboChart trigger: obstacle?obs
                # Save value communicated via the event 'obstacle' into obs variable, consuming
                # the event.
                self.obs = self.obstacle.take()
                self.logger.info("Transition on 'obstacle' event triggered to junction "
                                 f"with value '{self.obs}'")

                # Junction: a decision point, whose transitions are taken after the action #C of
                # the transition leading to it. Each transition out of the junction has a guard
                # and an action.
                #
                # Notice how here we have made the decision deterministic, whereas in the model
                # if obs==Direction.FRONT, either transition could be taken, so any is correct
                # with respecto the model.
                #
                # Both transitions lead to Rotating.
                @junction
                def j0():
                    if self.obs == Direction.RIGHT or self.obs == Direction.FRONT:
                        def action():
                            yield from self.move(0.0, avel)
                        return self.Rotating, action
                    elif self.obs == Direction.LEFT or self.obs == Direction.FRONT:
                        def action():
                            yield from self.move(0.0, -avel)
                        return self.Rotating, action

                # Transition obstacle?obs/#C to the junction, after Driving and its substates
                # have been exited.
                return j0, self.C.reset

    class Rotating(State):

        def entry(self):
            self.logger.info("In state 'Rotating'")

        # Just a transition with no guards, whose action calls rotate.
        def transitions(self):

            def action():
                # The operation continues executing till it finishes, so we delegate to it
                # with 'yield from'.
                yield from self.rotate(math.pi/5)

            return self.Driving, action


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

        # ROS2 Inputs
        self.subscriber = self.create_subscription(
            Obstacle, 'obstacle', self.obstacle_detected, 1)
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 1)

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
        self.get_logger().info("Received message on topic 'obstacle'")

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

        # Step the machine in response to the event, without stepping its clocks. Unless
        # consumed in this step, the event is then cleared.
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
