#!/usr/bin/python3
from typing import Tuple
import numpy as np

def ik2rr(x,y,gamma,l_a,l_b):
    r_sqr = x**2+y**2
    in_workspace = ((l_a-l_b)**2<=r_sqr) & (r_sqr <=(l_a+l_b)**2)
    if in_workspace:
        flag = True
        c_b = (r_sqr -l_a**2-l_b**2)/(2*l_a*l_b)
        if np.abs(c_b-1)<=0.0000001:
            c_b = 1
        elif np.abs(c_b+1)<=0.0000001:
            c_b = -1
        s_b = gamma*np.math.sqrt(1-c_b**2)
        q_a = np.math.atan2(y,x)-np.math.atan2(l_b*s_b,l_a+l_b*c_b)
        q_b = np.math.atan2(s_b,c_b)
    else:
        q_a = 0
        q_b = 0
        flag = False
    return q_a,q_b,flag

def inverse_kinematics(p:Tuple[float,float],gamma:Tuple[int,int],l_1:float,l_2:float,l_3:float,h_1:float,h_3:float)->Tuple[Tuple[float,float,float],int]:
    r_sqr = p[0]**2+p[1]**2
    r = np.math.sqrt(r_sqr)
    q_a,q_b,flag = ik2rr(gamma[0]*r-l_1,p[2]-h_1,-gamma[1],l_2,np.math.sqrt(l_3**2+h_3**2))
    if flag == True:
        q_1 = np.math.atan2(gamma[0]*p[1],gamma[0]*p[0])
        q_2 = q_a - np.math.pi/2
        q_3 = q_b + np.math.atan2(l_3,h_3)
    else:
        q_1 = 0
        q_2 = 0
        q_3 = 0
    return (q_1,q_2,q_3),flag