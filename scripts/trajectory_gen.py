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

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_position_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint_1','joint_2','joint_3']
        self.start_positions = [0.0,0.0,0.0]
        self.goal_positions = [0.5,0.50,0.5]
        self.setpoint_position = self.start_positions
        self.is_change_point = False
        self.i = 0


    def timer_callback(self):

        self.i = self.i + 1
        if self.i == 30:
            self.i = 0
            if self.is_change_point == False:
                self.is_change_point = True
                self.setpoint_position = self.start_positions
            else:
                self.is_change_point = False
                self.setpoint_position = self.goal_positions

        bazu_trajectory_msg = JointTrajectory()
        bazu_trajectory_msg.joint_names = self.joints
        ## creating a point
        point = JointTrajectoryPoint()
        point.positions = self.setpoint_position
        point.time_from_start = Duration(sec=1)
        ## adding newly created point into trajectory message
        bazu_trajectory_msg.points.append(point)
        # point.positions = self.goal_positions
        # point.time_from_start = Duration(sec=8)
        # bazu_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(bazu_trajectory_msg)
        print(self.setpoint_position)
 
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