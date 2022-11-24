#!/usr/bin/python3
import math
from re import S
import numpy as np
from numpy import ones, zeros
import rclpy
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        # publish_topic = "/joint_trajectory_position_controller/joint_trajectory"
        publish_topic = "/velocity_controllers/commands"
        self.vel_command_publihser = self.create_publisher(Float64MultiArray,publish_topic, 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speed_1 = [0.1,0.1,0.1]
        self.speed_2 = [-0.1,-0.1,-0.1]
        self.current_speed = self.speed_1
        self.is_change_point = False
        self.i = 0


    def timer_callback(self):
        self.i = self.i + 1
        if self.i == 30:
            self.i = 0
            if self.is_change_point == False:
                self.is_change_point = True
                self.current_speed = self.speed_1
            else:
                self.is_change_point = False
                self.current_speed = self.speed_2
        command = Float64MultiArray()
        command.data = self.current_speed
        self.vel_command_publihser.publish(command)
 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(joint_trajectory_object)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        joint_trajectory_object.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()