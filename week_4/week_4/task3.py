#!/usr/bin/env python3

import signal
from typing import Callable

from geometry_msgs.msg import Twist, TwistStamped, Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from week_4.robochart_fsm import Clock, Required, Scope, State, StateMachine

lvel = 0.1  # Linear velocity
avel = 0.0  # Angular velocity

class SampleForwardMachine(StateMachine):

    # Clocks
    clock_C: Clock

    # Provided by the robotic platform: requires IOperations
    cmd_vel: Required[Callable[[TwistStamped], None]]

    def initial(self):
        return self.Forward

    class Forward(State):

        def entry(self):
            # entry cmd_vel(TwistStamped(|
            #     header = default(),
            #     twist = Twist(| linear = Vector3(| x = lvel, y = 0, z = 0 |),
            #                     angular = Vector3(| x = 0, y = 0, z = avel |) |) |))
            #
            # Notice how here we use Header() as an implementation of the RoboChart
            # function default(), that is, we can use any value since the function
            # does not specify a particular value!
            self.cmd_vel(TwistStamped(header=Header(),
                                      twist=Twist(linear=Vector3(x=lvel, y=0.0, z=0.0),
                                                  angular=Vector3(x=0.0, y=0.0, z=avel))))
            self.logger.info("In state 'FORWARD'")

        # Because there are no transitions out of this state, we stay here forever, so there
        # is no need to define a 'transitions' method.
        #
        # Below, an example of using a clock to trigger a transition after 4 time units, with the
        # action #C, to a state TargetState:
        #
        # def transitions(self):
        #
        #     if self.clock_C.since() > 4:
        #         return self.TargetState, self.clock_C.reset
        #
        # Here, the convention is that the return value of transitions either returns the class
        # corresponding to a State on its own, or a tuple, such that the second component is
        # a Python method to be executed as an action, in this case, reseting of clock C.

# The robotic platform, providing the cmd_vel operation: provides IOperations
class TurtleBot3Platform(Scope):
    cmd_vel: Callable[[TwistStamped], None]

class ForwardMachineNode(Node):

    def __init__(self):
        super().__init__('sample_forward_machine')

        # Output
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Periodic callback for executing FSM
        self.timer_period = 0.1  # 100 milliseconds = 10 Hz for the control loop
        # 1 second is the value chosen for one 'time unit': a whole number of timer periods
        self.time_unit = 1

        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Setup of robotic platform, and of the state machine itself
        self.platform = TurtleBot3Platform(cmd_vel=self.cmd_vel)
        self.stm = SampleForwardMachine(self.get_logger(), self.timer_period, self.time_unit,
                                        parent=self.platform)

    def cmd_vel(self, msg):
        self.get_logger().info(
            f'Publishing cmd_vel: linear.x={msg.twist.linear.x}, angular.z={msg.twist.angular.z}')
        self.publisher.publish(msg)

    def control_loop(self):
        self.stm.execute()


def main(args=None):

    # On Ctrl+C (SIGINT), rclpy's own handler shuts down ROS, which stops spin below. Python
    # would also raise KeyboardInterrupt, wherever the program happens to be, so we disable that.
    signal.signal(signal.SIGINT, lambda signum, frame: None)

    rclpy.init(args=args)

    node = ForwardMachineNode()

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
