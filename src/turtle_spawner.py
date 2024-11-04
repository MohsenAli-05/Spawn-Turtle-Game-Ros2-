#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.get_logger().info("Turtle spawner has been started")
        self.spawn_turtles(5)

    def spawn_turtles(self, count):
        for i in range(count):
            x = random.uniform(0.0, 11.0) #Generate random position values for the turtles in the workspace 
            y = random.uniform(0.0, 11.0) #(workspace is from 0,0 to 11,11)
            angle = random.uniform(0.0, 2 * 3.14159)
            self.call_spawn_service(f"turtle{i+2}", x, y, angle)

    def call_spawn_service(self, name, x, y, theta):
        client = self.create_client(Spawn, "/spawn")

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for spawn service...")

        request = Spawn.Request()
        request.name = name
        request.x = x
        request.y = y
        request.theta = theta

        future = client.call_async(request)
        future.add_done_callback(self.callback_spawn)

    def callback_spawn(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned {response.name}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = TurtleSpawnerNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
