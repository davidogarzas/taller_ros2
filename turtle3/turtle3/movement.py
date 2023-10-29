#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
import math

from turtle_interfaces.msg import TurtleInfo
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.move_turtles_timer = self.create_timer(0.2,self.publish_cmd_vel)
        self.new_turtle_info_subscriber = self.create_subscription(TurtleInfo,"new_turtle_info",self.add_new_turtle_to_dictionary,10)
        self.delete_turtle_subscriber = self.create_subscription(TurtleInfo,"delete_from_dictionary",self.delete_turtle_from_dictionary,10)

        self.cmd_vel_publisher_dict = {}

    def add_new_turtle_to_dictionary(self,msg):
        vel = random.uniform(4.0,10.0)
        angle = random.uniform(-math.pi/4,math.pi/4)
        self.cmd_vel_publisher_dict[msg.name] = {
            "publisher": self.create_publisher(Twist,msg.name + "/cmd_vel",10),
            "vel": vel,
            "angle": angle,
            "x": msg.x,
            "y": msg.y,
            "theta": msg.theta,
            "flag": True
        }
        self.create_subscription(Pose,msg.name + "/pose",lambda new_msg: self.update_turtle_pose(new_msg,msg.name),10)

    def update_turtle_pose(self,msg,name):
        try:
            self.cmd_vel_publisher_dict[name]["x"] = msg.x
            self.cmd_vel_publisher_dict[name]["y"] = msg.y
            self.cmd_vel_publisher_dict[name]["theta"] = msg.theta
            if (msg.x < 1 or msg.x > 10 or msg.y < 1 or msg.y > 10) and self.cmd_vel_publisher_dict[name]["flag"] == True:
                self.cmd_vel_publisher_dict[name]["flag"] = False
                self.cmd_vel_publisher_dict[name]["angle"] = self.cmd_vel_publisher_dict[name]["angle"] + math.pi
            elif self.cmd_vel_publisher_dict[name]["flag"] == False and (msg.x > 1 and msg.x < 10 and msg.y > 1 and msg.y <10):
                self.cmd_vel_publisher_dict[name]["flag"] = True
                self.cmd_vel_publisher_dict[name]["angle"] = 0.0
        except:
            self.get_logger().info(name + " doesnt exist anymore")

    def publish_cmd_vel(self):
        msg = Twist()
        for publisher in self.cmd_vel_publisher_dict:
            try:
                msg.linear.x = self.cmd_vel_publisher_dict[publisher]["vel"]
                msg.angular.z = self.cmd_vel_publisher_dict[publisher]["angle"]
                self.cmd_vel_publisher_dict[publisher]["publisher"].publish(msg)
            except:
                self.get_logger().info(publisher + "doesnt exist anymore")

    def delete_turtle_from_dictionary(self,msg):
        del self.cmd_vel_publisher_dict[msg.name]

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
