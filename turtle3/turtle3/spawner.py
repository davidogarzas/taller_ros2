#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
import random
import math

from turtle_interfaces.msg import TurtleInfo
from turtlesim.srv import Spawn

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.spawn_turtle_timer = self.create_timer(0.75,self.generate_turtle_coords)
        self.turtle_info_publisher = self.create_publisher(TurtleInfo,"new_turtle_info",10)
        self.counter = 2
    
    def generate_turtle_coords(self):
        x = random.uniform(1.5,9.5)
        y = random.uniform(1.5,9.5)
        theta = random.uniform(-math.pi,math.pi)
        name = "turtle" + str(self.counter)
        self.counter += 1
        self.call_turtle_spawner_server(x,y,theta,name)

    def new_turtle_info_publisher(self,turtle):
        self.turtle_info_publisher.publish(turtle)

    def call_turtle_spawner_server(self,x,y,theta,name):
        client = self.create_client(Spawn,"spawn")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Server Turtle Spawner...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_turtle_spawner_server, x=x, y=y, theta=theta,name=name))

    def callback_turtle_spawner_server(self,future,x,y,theta,name):
        try:
            response = future.result()
            self.get_logger().info(name + " has been spawned")
            turtle = TurtleInfo()
            turtle.x = x
            turtle.y = y
            turtle.theta = theta
            turtle.name = name
            self.new_turtle_info_publisher(turtle)
            
        except Exception as e:
            self.get_logger().error("Service call failer %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
