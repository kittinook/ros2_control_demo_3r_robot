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
        self.goal_positions = [0.5,0.50,3.10]
        self.setpoint_position = self.start_positions
        self.is_change_point = False
        

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
        self.trajectory_publihser.publish(bazu_trajectory_msg)
    
    def translation(self, data, axis):
            H = np.identity(4)
            if axis == 'x':
                H[0,3] = data
            elif axis == 'y':
                H[1,3] = data
            elif axis == 'z':
                H[2,3] = data
            return H
    
    def rotation(self, data, axis):
        s = math.sin(data)
        c = math.cos(data)
        R = np.identity(4)
        if axis == 'x':
            R[1,1] = c
            R[1,2] = -s
            R[2,1] = s
            R[2,2] = c
        if axis == 'y':
            R[0,0] = c
            R[0,2] = s
            R[2,0] = -s
            R[2,2] = c
        if axis == 'z':
            R[0,0] = c
            R[0,1] = -s
            R[1,0] = s
            R[1,1] = c
        return R

    
    def fk(self, q):
        H_current = np.identity(4)
        H = np.array([np.identity(4), np.identity(4), np.identity(4), np.identity(4)])
        for i in range(len(q)):
            tx = self.translation(self.DH[i]["a"], 'x')
            rx = self.rotation(self.DH[i]["alpha"], 'x')
            tz = self.translation(self.DH[i]["d"], 'z')
            rz = self.rotation(self.DH[i]["theta"], 'z')
            H_current = H_current @ tx @ rx @ tz @ rz
            H[i,:,:] = H_current
        tx = self.translation(self.DH[i+1]["a"], 'x')
        rx = self.rotation(self.DH[i+1]["alpha"], 'x')
        tz = self.translation(self.DH[i+1]["d"], 'z')
        rz = self.rotation(self.DH[i+1]["theta"], 'z')
        H_current = H_current @ tx @ rx @ tz @ rz
        H[i+1, :, :] = H_current 
        print(H)
        R = H[:,:3,:3]
        P = H[:,:3,3]
        print(R)
        print(P)
        # R = H[3,:3,:3]
        # print(R)
        return H

    def endEffectorJacobianHW3(self, q):
        H = self.fk(q)
        joint_config = [1, 1, 1]
        R = H[:,:3,:3]
        P = H[:,:3,3]
        p_i = H[0,:3,3]
        j_w = []
        j_v = []
        for i in range(3):
            z_j = np.dot(R[i,:,:],np.array([0,0,1]))
            p_j = P[i,:]
            j_w.append(z_j.reshape((1,3)) * joint_config[i])
            j_v.append(np.cross(z_j, (p_i - p_j) + np.array((1-joint_config[i]) * z_j).reshape((1,3)))*joint_config[i])
        J_e = np.concatenate((np.array(j_w).transpose(), np.array(j_v).transpose()), axis=0).reshape((6,3))
        # np.set_printoptions(formatter={'float': '{: 0.4f}'.format}) # for visualization
        print(J_e)
        print(J_e[3:,:])
        return J_e

    def DIK(self, v):
        v = np.array([1,0,0])
        J_e = self.endEffectorJacobianHW3([0.1,0.8,0.4])
        J_e_reduce = J_e[3:,:]
        d = np.linalg.det(J_e_reduce)
        print(d)
        q_d = np.linalg.inv(J_e_reduce) @ v
        print(q_d)

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