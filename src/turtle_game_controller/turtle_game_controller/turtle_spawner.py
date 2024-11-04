#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
import random

from turtlesim.srv import SetPen
from functools import partial

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.get_logger().info("Turtle spawner has been started")
        
        self.active_turtles = {}  # dict for keeping the names of active turtles and their pos
    
        self.spawn_turtles(5)

    def spawn_turtles(self, count):
        self.call_set_pen_service(0,0,0,0,1) #turn off the pen
        for i in range(count):
            turtle_name = f"turtle{i+2}"  # Names each turtle
            self.spawn_turtle(turtle_name)
            # Subscribe to each turtle's pose to monitor if it's killed, to spawn a new one
            self.create_subscription(Pose, f"/{turtle_name}/pose", lambda msg, name=turtle_name: self.handle_turtle_pose(msg, name), 10)
            # W hena if we didn't use lambda, it would be a function call not a function reference and it wouldn't
            # wait for the Pose message to be recieved, would just execute the function immediately and won't take in the msg

    def spawn_turtle(self, name):
        # Generate random position values for the turtles in the workspace
        while True:
            x = random.uniform(0.5, 10.5) 
            y = random.uniform(0.5, 10.5) 
            x = max(0.5, min(10.5, x)) # Sometimes the code glitches and tries to spawn turtles
            y = max(0.5, min(10.5, y)) # outside the workspace, so this just ensures that it doesn't

            # Check if the new turtle's position is at least a unit away from existing turtles
            too_close = False
            for existing_turtle_name in self.active_turtles.keys():
                existing_pose = self.active_turtles[existing_turtle_name]  # Get the last known position of the existing turtle
                if existing_pose is not None:
                    distance = ((existing_pose.x - x) ** 2 + (existing_pose.y - y) ** 2) ** 0.5
                    if distance < 2:
                        too_close = True
                        break

            if not too_close:
                break

        angle = random.uniform(0.0, 2 * 3.14159)

        client = self.create_client(Spawn, "/spawn")

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for spawn service...")

        request = Spawn.Request()
        request.name = name
        request.x = x
        request.y = y
        request.theta = angle

        future = client.call_async(request)
        future.add_done_callback(lambda future: self.handle_spawn_response(future, name))
        # Hena estakhdemt lambda because we send a variable, I don't really get lambda, but
        # from what I could understand fel wa2t el 2a3adt dawart feeh en it's a way to create
        # a single use function that could take in multiple arguments that may be dynamic variables

    def handle_spawn_response(self, future, name):  # Handling error if failed to spawn turtle
        try:
            response = future.result()
            self.get_logger().info(f"Spawned {response.name}")
            self.active_turtles[name] = Pose()  # Initialize with an empty Pose
        except Exception as e:
            self.get_logger().error(f"Service call to spawn {name} failed: {e}")

    def handle_turtle_pose(self, msg, name):  # Callback for subscribing to turtle pose, in order to keep
        self.active_turtles[name] = msg  # Store the Pose message to track the turtle's position

    def monitor_turtles(self):  # Check which turtles are still active and respawn dead ones
        for name in list(self.active_turtles.keys()):
            # Check if the turtle is still active by validating its pose
            if self.active_turtles[name] is None:  # Assuming None means the turtle is dead
                self.get_logger().warn(f"{name} appears to be killed. Respawning...")
                self.spawn_turtle(name)
            # Reset the turtle's pose to None for the next check
            self.active_turtles[name] = None

    def start_monitoring(self):
        self.create_timer(0.5, self.monitor_turtles)  # Timer to check for missing turtles,
        # idk why the code was broken when this was merged with monitor_turtles()
    
    #-------------------------setting pen to clear-----------------------#
    def call_set_pen_service(self, r, g, b, width, off):
        client =self.create_client(SetPen, "/turtle1/set_pen")

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting For Service...")
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))
    
    def callback_set_pen(self, future):
        try: 
            response  = future.result()
        except Exception as e:
            self.get_logger.error("Service call failed: %r" % (e,))
        

def main(args=None):
    rclpy.init(args=args)

    node = TurtleSpawnerNode()
    node.start_monitoring()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
