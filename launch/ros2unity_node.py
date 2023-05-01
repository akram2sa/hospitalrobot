#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('my_robot_controller')
        
        # Subscribe to the '/Coordinates' topic
        self.subscription = self.node.create_subscription(
            Vector3,
            '/Coordinates',
            self.callback,
            rclpy.qos.qos_profile_system_default)

    def callback(self, msg):
        # Translate the robot based on the received message
        translation = msg.x, msg.y, msg.z
        # Set the translation of the robot here

    def spin(self):
        while rclpy.ok():
            # Perform robot movements here
            rclpy.spin_once(self.node, timeout_sec=0.1)
            rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()
def main():
    controller = RobotController()
    controller.spin()

if __name__ == '__main__':
    main()