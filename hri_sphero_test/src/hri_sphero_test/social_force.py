#!/usr/bin/python

import rospy

import math
import sys
import tf
import PyKDL
import numpy as np
import time

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

SCALE = 40
IS_ROS_NODE = False
CONTROLLED_AGENT = 'sphero_ggw'
STOP_DISTANCE = 0.2

def pairs(lst):
    i = iter(lst)
    first = prev = item = i.next()
    for item in i:
        yield prev, item
        prev = item
    yield item, first

class Sphero(object):
    # Force model constants:    
    V0 = 200.0 # interagent
    sig = 1
    U0 = 500.0 # bound
    R = 0.05

    # Boundary constants:
    #xlim = np.array((-3,0),(3,0))
    #ylim = np.array((0,-3),(0,3))
    # changed limit to x = [0,3] and y = [0,3]
    #bounds = [(-3,0),(3,0),(0,-3),(0,3)] # format: (x,y)
    bounds = [(0,0),(3,0),(3,3),(0,3)] # format: (x,y)

    # Clamping the velocity
    v_clamp = 30   

    def __init__(self, number, name):
        self.xpos = None#0.0
        self.ypos = None#0.0
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
        if IS_ROS_NODE: self.cmd_vel_pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size = 10)
        if IS_ROS_NODE: self.color_pub = rospy.Publisher(self.name+'/set_color', ColorRGBA, queue_size = 10)
        if IS_ROS_NODE: rospy.Subscriber(self.name+'/global_pose', geometry_msgs.msg.Pose, self.set_pose)

         # Inertia, rate constants
        self.mass =  1.0
        self.rate = 3 # update rate in force-momentum equation; dt = 1/rate; F*dt = mass*dv
        if IS_ROS_NODE: self.time = rospy.Time.now().to_sec()

        self.xmax = max(corner[0] for corner in self.bounds)
        self.ymax = max(corner[1] for corner in self.bounds)
        self.xmin = min(corner[0] for corner in self.bounds)
        self.ymin = min(corner[1] for corner in self.bounds)
    
    def set_wp(self,data):
        # Set the waypointing direction and magnitude for a sphero:
        self.w_force_mag = data
        if data[0] > 0:
            self.wp_points = np.array([2.5,1.5]) # single point WP for rightward agents
        else:
            self.wp_points = np.array([0.5,1.5]) # for leftward agents


    def bound_repulse(self):
        eps = np.finfo(float).eps
        b_force_x = 0.0
        b_force_y = 0.0
        for ((xlim_1, ylim_1), (xlim_2, ylim_2)) in pairs(self.bounds):
            #print("Limits are :" + str(xlim) +" , " + str(ylim))
            if xlim_1 == xlim_2: # different y
                if self.xmax <= self.xpos:
                    b_force_x -= (self.U0)*0.2
                elif self.xmin >= self.xpos:
                    b_force_x += (self.U0)*0.2
                else:
                    bx = math.sqrt((self.xpos - xlim_1)**2) + eps
                    b_force_x += (self.U0/self.R)* math.exp(-bx/self.R)*(self.xpos - xlim_1)/bx

            if ylim_1 == ylim_2: # different x
                if self.ymax <= self.ypos:
                    b_force_y -= (self.U0)*0.2
                elif self.ymin >= self.ypos:
                    b_force_y += (self.U0)*0.2
                else:
                    by = math.sqrt((self.ypos - ylim_1)**2) + eps
                    b_force_y += (self.U0/self.R)* math.exp(-by/self.R)*(self.ypos - ylim_1)/by

        self.b_force[0] = b_force_x
        self.b_force[1] = b_force_y

    def agent_repulse(self, other):
        eps = np.finfo(float).eps
        other_x,other_y = other.ret_pos()
        V0 = self.V0
        sig = self.sig
        b = math.sqrt((self.xpos - other_x)**2 + (self.ypos - other_y)**2) + eps
        self.a_force += (V0/sig)*(math.exp(-b/sig))*np.array([self.xpos - other.xpos , self.ypos - other.ypos])/b        
        
    def wp_attraction(self):
        eps = np.finfo(float).eps
        w_mag = math.fabs(self.w_force_mag[0])
        b = math.sqrt((self.xpos - self.wp_points[0])**2 + (self.ypos - self.wp_points[1])**2) + eps
        self.w_force = -w_mag*np.array([self.xpos - self.wp_points[0] , self.ypos - self.wp_points[1]])/b
    
        
    def update_velocity(self):
        coefficient = 1
        if abs(1.5-self.xpos) > 1.5:
            coefficient = -1
        corrected_w_force = coefficient* self.w_force
        total_force = self.a_force + self.b_force + corrected_w_force
        #dt = rospy.Time.now().to_sec() - self.time
        #print total_force
        #self.xvel += (total_force[0]/(self.mass)) * dt
        #self.yvel += (total_force[1]/(self.mass)) * dt
        #!!!!!! from 30 to 40
        #!!!!!! CAP xvel, yvel 30
        # mapping of velocity
        self.xvel += (total_force[0]/(self.mass*self.rate))
        self.yvel += (total_force[1]/(self.mass*self.rate))     

        v_clamp = self.v_clamp

        if self.xvel >= v_clamp:        
            self.xvel = v_clamp
        elif self.xvel <= -v_clamp:
            self.xvel = -v_clamp
        if self.yvel >= v_clamp:        
            self.yvel = v_clamp
        elif self.yvel <= -v_clamp:
            self.yvel = -v_clamp

        self.speed = math.sqrt(self.xvel**2 + self.yvel**2)
        self.heading = math.atan2(self.ypos,self.xpos)

        rospy.loginfo(self.name +' xpos: {0}, ypos: {1}'.format(self.xpos, self.ypos))
        rospy.loginfo(self.name +' b_force: {0}, total_force: {1}'.format(self.b_force, total_force))
        rospy.loginfo(self.name +' xvel: {0}, yvel: {1}'.format(self.xvel, self.yvel))
        if IS_ROS_NODE: self.time = rospy.Time.now().to_sec()

        return total_force

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
        self.update_velocity()
        
        # vx = self.xvel/math.sqrt(self.xvel**2 + self.yvel**2)*SCALE
        # vy = self.yvel/math.sqrt(self.xvel**2 + self.yvel**2)*SCALE
        
        # Do a linear mapping instead of scaling: from 30 to 40 in each case
        v_l = 45 # Lower and upper limits for the linear mapping
        v_h = 55
        v_clamp = self.v_clamp
        if self.xvel == 0:
            vx = 0
        elif self.xvel > 0:
            vx = v_l + (v_h-v_l)*self.xvel/(v_clamp)
        else:
            vx = -v_l + (v_h-v_l)*self.xvel/(v_clamp) 

        if self.yvel == 0:
            vy = 0
        elif self.yvel > 0:
            vy = v_l + (v_h-v_l)*self.yvel/(v_clamp)
        else:
            vy = -v_l + (v_h-v_l)*self.yvel/(v_clamp) 
        
        # Just for simulation. Comment out the vx2,vy2 publisher for real tests
        v_l2 = 0
        if self.xvel >= 0:
            vx2 = v_l2 + (v_h-v_l2)*self.xvel/(v_clamp)
        else:
            vx2 = -v_l2 + (v_h-v_l2)*self.xvel/(v_clamp)

        if self.yvel >= 0:
            vy2 = v_l2 + (v_h-v_l2)*self.yvel/(v_clamp)
        else:
            vy2 = -v_l2 + (v_h-v_l2)*self.yvel/(v_clamp) 
        
        # check if we are close enough
        if math.sqrt((self.xpos-self.wp_points[0])**2 + (self.ypos-self.wp_points[1])**2) \
            < STOP_DISTANCE:
            self.set_wp(-1*self.w_force_mag)
            rospy.loginfo(self.name+" switching direction")
        #    vx, vy = 0,0
        #    rospy.loginfo(self.name+' is stopping. Reached destination.')


        #if self.name == 'sphero_rgw':
            #rospy.loginfo('vx: {0}, vy: {1}'.format(vx, vy))
        #rospy.loginfo(self.name+ ' vx2: {0}, vy2: {1}'.format(vx2, vy2))
        rospy.loginfo(self.name+' vx: {0}, vy: {1}'.format(vx, vy))

        if self.name != CONTROLLED_AGENT:
            self.vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(vx,vy,0), geometry_msgs.msg.Vector3(0,0,0))
            #self.vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(vx2,vy2,0), geometry_msgs.msg.Vector3(0,0,0))
            self.cmd_vel_pub.publish(self.vel_msg)
            # Send updated velocity to Sphero

    def pub_color(self,r,g,b,a):
        color_msg = ColorRGBA()
        color_msg.r = r
        color_msg.g = g
        color_msg.b = b
        color_msg.a = a
        self.color_pub.publish(color_msg)

        
if __name__ == '__main__':
    #sphero_list = ['sphero_rgw', 'sphero_wrb']
    #N_agents = 2
    #sphero_list = ['sphero_rgw', 'sphero_wrb', CONTROLLED_AGENT]
    #N_agents = 3
    sphero_list = ['sphero_rgw', 'sphero_wrb', 'sphero_yoo', 'sphero_wpp', CONTROLLED_AGENT]
    N_agents = 5
    spheros = []
    w_force = 100 #100
    if IS_ROS_NODE: rospy.init_node('social_force')
    # Initialize spheros: 
    for i in range(0,N_agents):
        name_i = sphero_list[i]#'name' + str(i)
        if i%2 == 0:
            wp_i = np.array([w_force,0])
        else:
            wp_i = np.array([-w_force,0])
        spheros.append(Sphero(i,name_i))
        spheros[i].set_wp(wp_i)

    r = spheros[0].rate
    #r = 10 # rate of 100 Hz

    if IS_ROS_NODE:
        rate = rospy.Rate(r)

        while not rospy.is_shutdown():

            # make sure we get pose before moving
            if not all([obj.xpos for obj in spheros] + [obj.ypos for obj in spheros]):
                print [obj.xpos for obj in spheros] + [obj.ypos for obj in spheros]
                time.sleep(0.2)
                continue

            # Publish color
            for obj_idx, obj in enumerate(spheros):
                if obj.name != CONTROLLED_AGENT:
                    if obj.w_force_mag[0] > 0:
                        obj.pub_color(255,0,0,0)
                    else:
                        obj.pub_color(0,0,255,0)

            # Publish velocities based on the force:
            for obj in spheros:
                obj.pub_cmd()

            # Calculate boundary forces:
            for obj in spheros:
                obj.bound_repulse()
                obj.wp_attraction()
                obj.a_force = np.array([0.0,0.0])
                for obj2 in spheros:
                    if obj2 != obj:
                        obj.agent_repulse(obj2)

            rate.sleep()
    else:
        import numpy as np
        import matplotlib.pyplot as plt
        import matplotlib.lines as mlines
        # Set limits and number of points in grid
        SPACING_Y = 13j
        SPACING_X = 13j
        y, x = np.mgrid[0:3:SPACING_Y, 0:3:SPACING_X]
        print("x: [\n"+'\n'.join(" ".join('{:7.4f}'.format(k) for k in x_list) for x_list in x)+"]\n")
        print("y: [\n"+'\n'.join(" ".join('{:7.4f}'.format(k) for k in y_list) for y_list in y)+"]\n")

        fy, fx = np.mgrid[0:0:SPACING_Y, 0:0:SPACING_X]
        obj = spheros[0]

        # force init pos for now
        sphero_list = ['sphero_rgw', 'sphero_wrb', 'sphero_yoo', 'sphero_wpp', CONTROLLED_AGENT]

        obj_wrb = spheros[1]
        obj_wrb.xpos = 2.5
        obj_wrb.ypos = 1.4
        obj_yoo = spheros[2]
        obj_yoo.xpos = 0.8
        obj_yoo.ypos = 2.5
        obj_wpp = spheros[3]
        obj_wpp.xpos = 2.0
        obj_wpp.ypos = 0.5
        obj_ggw = spheros[4]
        obj_ggw.xpos = 0.5
        obj_ggw.ypos = 0.5
        for x_row in x:
            for idx_x_row, x_coord in enumerate(x_row):
                for idx_y_row, y_row in enumerate(y):
                    for y_coord in y_row:
                        obj.xpos = x_coord
                        obj.ypos = y_coord
                        obj.bound_repulse()
                        obj.wp_attraction()
                        obj.a_force = np.array([0.0,0.0])
                        for obj2 in spheros:
                            if obj2 != obj:
                                obj.agent_repulse(obj2)

                        total_force = obj.update_velocity()
                        fy[idx_y_row][idx_x_row] = total_force[1]#idx_y_row
                        fx[idx_y_row][idx_x_row] = total_force[0]#idx_x_row

        print("fx: [\n"+'\n'.join(" ".join('{:7.4f}'.format(k) for k in fx_list) for fx_list in fx)+"]\n")
        print("fy: [\n"+'\n'.join(" ".join('{:7.4f}'.format(k) for k in fy_list) for fy_list in fy)+"]\n")


        fs = np.array([math.sqrt(fx_component**2 + fy_component**2) for fx_component, fy_component in zip(np.ravel(fx), np.ravel(fy))])
        f = np.flipud(fs.reshape(fx.shape))

        # plot!
        fig, ax = plt.subplots()
        im = ax.imshow(f, extent=[x.min(), x.max(), y.min(), y.max()])
        fig.colorbar(im)

        ax.quiver(x, y, fx, fy, units='height')
        ax.set(aspect=1, title='Force Field', xlabel='x(m)', ylabel='y(m)')

        # plot robot location
        obj_rgw = spheros[0]
        obj_rgw.xpos = 0.5
        obj_rgw.ypos = 1.6
        color = ['r','g','y','w','k']
        #ax.plot(obj.xpos, obj.ypos,  c=color[0], marker='o', markersize=20, label=sphero_list[0])
        for idx, obj2 in enumerate(spheros):
            ax.plot(obj2.xpos, obj2.ypos,  c=color[idx], marker='o', markersize=20, label=sphero_list[idx])
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=3, numpoints=1)
        plt.show()