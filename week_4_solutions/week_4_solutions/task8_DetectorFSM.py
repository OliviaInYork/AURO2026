#!/usr/bin/env python3

# Sample solution for Week4's task 4.
#
# This Python program implements the TurtleBot3FSMC.

from enum import Enum
import signal

from auro_interfaces.msg import Obstacle
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
# RoboChart specific implementations
from week_4.robochart_fsm import Input, junction, Output, Required, Scope, State, StateMachine
from week_4.robochart_sequence_toolkit import extract, ran, Seq
from week_4.robochart_set_toolkit import interval, union

# Constant defined in DetectorFSM.
SCAN_THRESHOLD = 0.4

class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    FRONT = 2

    @classmethod
    def default(cls):
        return cls.LEFT  # Provide a default value


# We define DetectorFSM by inheriting from StateMachine
class DetectorFSM(StateMachine):

    # Variables
    data: Seq[float] = Seq([])

    # Provided by the robotic platform: the input event scan, and the output event obstacle
    scan: Required[Input[Seq[float]]]
    obstacle: Required[Output[Direction]]

    def initial(self):
        return self.Idle

    class Idle(State):

        # No entry action
        def entry(self):
            self.logger.info("In state 'Idle'")

        def transitions(self):
            if self.scan.triggered():

                self.logger.info("Transition on 'scan' event triggered to junction")

                # Implementation of RoboChart trigger: scan?data
                # Save value communicated via the event 'scan' into data variable, consuming
                # the event.
                self.data = self.scan.take()

                # Junction: a decision point, whose transitions are taken in order. Each
                # transition out of the junction has a guard, and all lead back to Idle. If no
                # guard holds, the machine waits in the junction, and cannot be interrupted. Since
                # the guards only depend on data, which does not change, it is deadlocked there.
                @junction
                def j0():
                    # Indices of sequences start at 1, as in RoboChart.
                    front = min(ran(extract(union(interval(345, 359), interval(1, 14)),
                                            self.data)))
                    left = min(ran(extract(interval(16, 43), self.data)))
                    right = min(ran(extract(interval(315, 343), self.data)))

                    if front < SCAN_THRESHOLD:
                        self.logger.info("obstacle: front")
                        def action():
                            self.obstacle(Direction.FRONT)
                        return self.Idle, action
                    elif left < SCAN_THRESHOLD:
                        self.logger.info("obstacle: left")
                        def action():
                            self.obstacle(Direction.LEFT)
                        return self.Idle, action
                    elif right < SCAN_THRESHOLD:
                        self.logger.info("obstacle: right")
                        def action():
                            self.obstacle(Direction.RIGHT)
                        return self.Idle, action
                    else: # None apply, so back to idle with no action
                        return self.Idle

                # Transition scan?data to the junction
                return j0


# The robotic platform: its events.
class TurtleBot3Platform(Scope):
    scan: Input[Seq[float]]
    obstacle: Output[Direction]


class DetectorFSMNode(Node):

    def __init__(self):
        super().__init__('DetectorFSMNode')

        # ROS2 Outputs
        self.publisher = self.create_publisher(Obstacle, 'obstacle', 1)

        # ROS2 Inputs
        self.subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)

        # Periodic callback for executing FSM
        self.timer_period = 0.1  # 100 milliseconds = 10 Hz for the control loop
        # 1 second is the value chosen for one 'time unit': a whole number of timer periods
        self.time_unit = 1

        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Setup of robotic platform, whose output event obstacle is implemented by send_obstacle
        self.platform = TurtleBot3Platform(obstacle=self.send_obstacle)

        # Setup the state machine itself. This checks that everything it requires is provided.
        self.stm = DetectorFSM(self.get_logger(), self.timer_period, self.time_unit,
                               parent=self.platform)

    def send_obstacle(self, value):
        msg = Obstacle()

        # Mapping between model types and ROS2 Obstacle message type
        match value:
            case Direction.FRONT:
                msg.data = Obstacle.FRONT
            case Direction.LEFT:
                msg.data = Obstacle.LEFT
            case Direction.RIGHT:
                msg.data = Obstacle.RIGHT

        #self.get_logger().info(f'Publishing obstacle: {msg}')
        self.publisher.publish(msg)

    def scan_callback(self, msg):
        # self.get_logger().info("Received message on topic 'scan'")

        # We set the event in the platform, passing the data, mapping from a ROS message
        # type, in this case, LaserScan, to a type that corresponds to the sequence of reals
        # in RoboChart.
        self.platform.scan.set(Seq(msg.ranges))

        # Step the machine in response to the event, without stepping its clocks. Unless
        # consumed in this step, the event is then cleared.
        self.stm.execute(timed=False)

    def control_loop(self):

        # We step the machine
        self.get_logger().debug('-- Executing control_loop. --')
        self.stm.execute()


def main(args=None):

    # On Ctrl+C (SIGINT), rclpy's own handler shuts down ROS, which stops spin below. Python
    # would also raise KeyboardInterrupt, wherever the program happens to be, so we disable that.
    signal.signal(signal.SIGINT, lambda signum, frame: None)

    rclpy.init(args=args)

    node = DetectorFSMNode()

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
