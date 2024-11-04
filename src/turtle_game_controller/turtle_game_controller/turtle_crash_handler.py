#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill
from turtlesim.msg import Pose
import math

class TurtleCollisionDetectorNode(Node):
    def __init__(self):
        super().__init__("turtle_collision_detector")
        self.get_logger().info("Turtle collision detector has been started")

        # Dictionary to store the positions of all turtles
        self.turtle_positions = {}

        # Subscribe to the main turtle's position
        self.create_subscription(Pose, "/turtle1/pose", self.update_main_turtle_position, 10)

        # Subscribe to each newly spawned turtle's position with a lambda function to include the turtle name
        for i in range(2, 7):  # Since turtle_spawner spawns turtles "turtle2" to "turtle6"
            topic_name = f"/turtle{i}/pose"
            self.create_subscription(Pose, topic_name, lambda msg, name=f"turtle{i}": self.update_spawned_turtle_position(msg, name), 10)

    def update_main_turtle_position(self, msg):
        self.turtle_positions["turtle1"] = (msg.x, msg.y)
        self.check_collisions()

    def update_spawned_turtle_position(self, msg, name):
        self.turtle_positions[name] = (msg.x, msg.y)

    def check_collisions(self):
        if "turtle1" not in self.turtle_positions:
            return

        main_x, main_y = self.turtle_positions["turtle1"]

        # Check distance between main turt and other turts
        for name, (x, y) in list(self.turtle_positions.items()):
            if name != "turtle1":
                distance = math.sqrt((main_x - x) ** 2 + (main_y - y) ** 2)
                if distance < 0.7: 
                    self.get_logger().info(f"Collision detected with {name}")
                    self.kill_turtle(name)

    def kill_turtle(self, name):
        client = self.create_client(Kill, "/kill")

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for kill service...")

        request = Kill.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(lambda future: self.callback_kill(name, future))

    def callback_kill(self, name, future):
        try:
            future.result()
            self.get_logger().info(f"Killed {name}")
            # Remove the turtle's position from tracking after killing it
            self.turtle_positions.pop(name, None)
        except Exception as e:
            self.get_logger().error(f"Failed to kill {name}: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = TurtleCollisionDetectorNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
