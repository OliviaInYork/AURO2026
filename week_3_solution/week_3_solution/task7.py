#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import Float64MultiArray

from transforms3d.euler import euler2quat

class Task7(Node):
    def __init__(self):
        super().__init__('task7')
        
        # parameters
        self.L1 = float(self.declare_parameter('L1', 0.4).value)
        self.L2 = float(self.declare_parameter('L2', 0.3).value)

        # proportional co-efficient Kp
        self.Kp = float(self.declare_parameter('Kp', 0.01).value)

        # desired angle
        self.desired = None

        # subscribers
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_cb, 10)
        self.command_sub = self.create_subscription(Float64MultiArray, '/position_controller/commands', self.command_cb, 10)

        # publishers
        self.command_pub = self.create_publisher(Float64MultiArray, '/joint_controller/commands', 10)

        self.idx2 = None

    def command_cb(self, msg: Float64MultiArray):

        try:
            self.desired = msg.data[1]
        except ValueError:
            self.get_logger().error("Received message on '/position_controller/commands' with wrong number of values.")

    def joint_cb(self, msg: JointState):

        # Find indices by name once
        if self.idx2 is None:
            try:
                self.idx2 = msg.name.index('joint2')
            except IndexError:
                self.get_logger().warn("Expected joint name 'joint2' in /joint_states")
                return

        if self.desired is not None:    

            # Get joint position
            q2 = float(msg.position[self.idx2])
            
            # Calculate error
            e = self.desired-q2

            # Output torque computed from Kp * e
            T = (self.Kp * e)

            # build array data
            arr = Float64MultiArray()
            arr.data = [0.0, T]

            # debug info
            self.get_logger().info(f"desired: {self.desired}, actual: {q2}, error: {e}, command: {arr.data}")

            # publish
            self.command_pub.publish(arr)

def main():
    rclpy.init()
    rclpy.spin(Task7())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
