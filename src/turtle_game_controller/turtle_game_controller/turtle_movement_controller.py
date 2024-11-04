#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import sys
import tty
import termios

class TurtleMovementNode(Node):
    def __init__(self):
        super().__init__("turtle_teleop")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.call_set_pen_service(0, 0, 0, 3, 1)  # Turn off the pen
        self.get_logger().info("Turtle teleop has been started. Use arrow keys to move.")
        self.move_turtle()  # Start moving the turtle immediately

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for SetPen service...")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(self.callback_set_pen)

    def callback_set_pen(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to set pen: {e}")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(3)  # Read 3 characters for arrow keys
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def move_turtle(self):
        twist = Twist()
        while rclpy.ok():  # Keep running while ROS is still running
            key = self.get_key()
            if key == '\x1b[A':  # Up arrow
                twist.linear.x = 1.0
                twist.angular.z = 0.0
            elif key == '\x1b[B':  # Down arrow
                twist.linear.x = -1.0
                twist.angular.z = 0.0
            elif key == '\x1b[C':  # Right arrow
                twist.linear.x = 0.0
                twist.angular.z = -1.0
            elif key == '\x1b[D':  # Left arrow
                twist.linear.x = 0.0
                twist.angular.z = 1.0
            else:  # Stop moving
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.cmd_vel_publisher_.publish(twist)
            rclpy.spin_once(self)  # Allow ROS to process other callbacks

def main(args=None):
    rclpy.init(args=args)

    try:
        node = TurtleMovementNode()
        rclpy.spin(node)  # Keep the node alive
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
