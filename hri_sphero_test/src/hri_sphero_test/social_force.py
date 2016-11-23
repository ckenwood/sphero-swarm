#!/usr/bin/python

import rospy

import math
import sys
import tf
import PyKDL
import numpy as np

from sphero_driver import sphero_driver
import dynamic_reconfigure.server
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from sphero_node.msg import SpheroCollision
from std_msgs.msg import ColorRGBA, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sphero_node.cfg import ReconfigConfig

SCALE = 50
class Sphero(object):
    # Force model constants:    
    V0 = 1.0
    R = 1.0
    U0 = 1.0
    sig = 1.0

    # Boundary constants:
    #xlim = np.array((-3,0),(3,0))
    #ylim = np.array((0,-3),(0,3))
    bounds = [(-3,0),(3,0),(0,-3),(0,3)] # format: (x,y)
   
    def __init__(self, number, name):
        self.xpos = 0.0
        self.ypos = 0.0
        self.xvel = 0.0
        self.yvel = 0.0
        self.vx_imu = 0.0
        self.vy_imu = 0.0
        self.b_force = np.array([0.0, 0.0]) # Boundary repulsion
        self.a_force = np.array([0.0, 0.0]) # Inter-agent repulsion
        self.w_force = np.array([1.0, 0.0]) # Waypoint finding force
        self.speed = 0.0
        self.heading = 0.0
        self.number = number
        self.name = name
        self.cmd_vel_pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size = 10)
         # Inertia, rate constants
        self.mass =  1.0
        self.rate = 100 # update rate in force-momentum equation; dt = 1/rate; F*dt = mass*dv

    
    def set_wp(self,data):
        # Set the waypointing direction and magnitude for a sphero:
        self.w_force = data

    def bound_repulse(self):
        eps = np.finfo(float).eps
        b_force_x = 0.0
        b_force_y = 0.0
        for (xlim,ylim) in self.bounds:
            #print("Limits are :" + str(xlim) +" , " + str(ylim))
            bx = math.sqrt((self.xpos - xlim)**2) + eps
            by = math.sqrt((self.ypos - ylim)**2) + eps
            if xlim != 0.0:
                b_force_x += (self.U0/self.R)* math.exp(-bx/self.R)*(self.xpos - xlim)/bx
            if ylim != 0.0:
                b_force_y += (self.U0/self.R)* math.exp(-by/self.R)*(self.ypos - ylim)/by

        self.b_force[0] = b_force_x
        self.b_force[1] = b_force_y

    def agent_repulse(self, other):
        eps = np.finfo(float).eps
        other_x,other_y = other.ret_pos()
        V0 = self.V0
        sig = self.sig
        b = math.sqrt((self.xpos - other_x)**2 + (self.ypos - other_y)**2) + eps
        self.a_force = (V0/sig)*(math.exp(-b/sig))*np.array([self.xpos - other.xpos , self.ypos - other.ypos])/b

    def update_velocity(self):
        total_force = self.a_force + self.b_force + self.w_force
        #print total_force
        self.xvel += (total_force[0]/(self.mass*self.rate))
        self.yvel += (total_force[1]/(self.mass*self.rate))
        self.speed = math.sqrt(self.xvel**2 + self.yvel**2)
        self.heading = math.atan2(self.ypos,self.xpos)
        rospy.loginfo('b_force: {0}, total_force: {1}'.format(self.b_force, total_force))

    def set_pose(self, msg):
        self.xpos = msg.position.x
        self.ypos = msg.position.y
        #self.vx_imu = msg.twist.linear.x
        #self.vy_imu = msg.twist.linear.y
        #self.xvel = msg.twist.linear.x
        #self.yvel = msg.twist.linear.y

    # def _init_pubsub(self):
    #     self.cmd_vel_pub = rospy.Publisher(self.name + 'cmd_vel', Twist, queue_size = 10)
    
    def ret_pos(self):
        return (self.xpos,self.ypos)
    

    def pub_cmd(self):
        rospy.Subscriber(self.name + '/sphero_pose', geometry_msgs.msg.Pose, self.set_pose)
        self.update_velocity()
        vx = self.xvel/math.sqrt(self.xvel**2 + self.yvel**2)*SCALE
        vy = self.yvel/math.sqrt(self.xvel**2 + self.yvel**2)*SCALE
        self.vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(vx,vy,0), geometry_msgs.msg.Vector3(0,0,0))
        self.cmd_vel_pub.publish(self.vel_msg)
        # Send updated velocity to Sphero



        
if __name__ == '__main__':
    sphero_list = ['sphero_ypw', 'sphero_wrb']
    N_agents = 2
    spheros = []
    rospy.init_node('social_force')
    # Initialize spheros: 
    for i in range(0,N_agents):
        sphero_list[i]
        name_i = sphero_list[i]#'name' + str(i)
        if i%2 == 0:
            wp_i = np.array([1,0])
        else:
            wp_i = np.array([-1,0])
        spheros.append(Sphero(i,name_i))
        spheros[i].set_wp(wp_i)

    r = 100 # rate of 100 Hz
    rate = rospy.Rate(r)
    while not rospy.is_shutdown():

        # Publish velocities based on the force: 
        for obj in spheros:
            obj.pub_cmd()
    
        # Calculate boundary forces:
        for obj in spheros:
            obj.bound_repulse()
            for obj2 in spheros:
                if obj2 != obj:
                    obj.agent_repulse(obj2)

        rate.sleep()
