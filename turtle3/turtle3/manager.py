#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
import random
import math

from turtle_interfaces.msg import TurtleInfo
from turtle_interfaces.msg import TurtleArray
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.new_turtle_info_subscriber = self.create_subscription(TurtleInfo,"new_turtle_info",self.add_new_turtle_to_dictionary,10)
        self.turtle_target_publisher = self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.delete_turtle_from_dictionary_publisher = self.create_publisher(TurtleInfo,"delete_from_dictionary",10)
        self.create_subscription(Pose,"turtle1/pose",self.callback_turtle1_pose,10)
        self.cmd_vel_turtle1_timer = self.create_timer(0.01,self.calculate_target)
        self.current_pose = None
        self.turtle_dictionary = {}

    def add_new_turtle_to_dictionary(self,msg):
        self.turtle_dictionary[msg.name] = msg
        self.create_subscription(Pose,msg.name + "/pose",lambda new_msg: self.callback_turtle_pose(new_msg,msg.name),10)

    def callback_turtle_pose(self,msg,name):
        try:
            self.turtle_dictionary[name].x = msg.x
            self.turtle_dictionary[name].y = msg.y
            self.turtle_dictionary[name].theta = msg.theta
        except:
            self.get_logger().info(name + " doesnt exist anymore")


    def callback_turtle1_pose(self,msg):
        self.current_pose = msg
        
    def calculate_target(self):
        closest_turtle = None
        closest_distance = [20,20,150]
        msg = Twist()

        if self.current_pose != None and self.turtle_dictionary:
            for turtle in self.turtle_dictionary:
                dist_x = self.turtle_dictionary[turtle].x - self.current_pose.x
                dist_y = self.turtle_dictionary[turtle].y - self.current_pose.y
                dist = math.sqrt(dist_x*dist_x + dist_y*dist_y)
                if closest_turtle != turtle and dist < closest_distance[2]:
                    closest_turtle = turtle
                    closest_distance = [dist_x,dist_y,dist]
            
            if closest_distance[2] > 0.5:
                theta = math.atan2(closest_distance[1],closest_distance[0])
                diff_theta = theta - self.current_pose.theta

                if diff_theta > math.pi:
                    diff_theta -= 2*math.pi
                elif diff_theta < -math.pi:
                    diff_theta += 2*math.pi

                msg.linear.x = 10.0
                msg.angular.z = 10*diff_theta
            
            else:
                delete = TurtleInfo()
                delete.name = closest_turtle
                self.delete_turtle_from_dictionary_publisher.publish(delete)
                del self.turtle_dictionary[closest_turtle]
                self.call_kill_turtle_server(closest_turtle)
                msg.linear.x = 0.0
                msg.angular.z = 0.0

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        self.turtle_target_publisher.publish(msg)

    def call_kill_turtle_server(self,name):
        client = self.create_client(Kill,"kill")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Server Turtle Killer...")

        request = Kill.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill_turtle_server, name = name))

    def callback_kill_turtle_server(self,future,name):
        try:
            response = future.result()
            self.get_logger().info(name + " has been killed")
        except Exception as e:
            self.get_logger().error("Service call failer %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
