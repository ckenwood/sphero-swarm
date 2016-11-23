import rospy
import numpy
import math

from math import sin, cos

def global2local(global_pose, robot_pose):
    #robot_pose = [x,y,theta]
    #global_pose = [x,y]

    [x,y,theta] = robot_pose
    [global_x, global_y] = global_pose

    RTMatrix = numpy.matrix([[cos(theta), -sin(theta),x],[sin(theta),cos(theta),y],[0,0,1]])
    local_pose = RTMatrix.I*numpy.matrix([global_x, global_y,1]).T
    return local_pose.T.tolist()[0][0:2]

def local2global(local_pose, robot_pose):
    #robot_pose = [x,y,theta]
    #local_pose = [x,y]

    [x,y,theta] = robot_pose
    [local_x, local_y] = local_pose

    RTMatrix = numpy.matrix([[cos(theta), -sin(theta),x],[sin(theta),cos(theta),y],[0,0,1]])
    global_pose = RTMatrix*numpy.matrix([local_x, local_y,1]).T
    return global_pose.T.tolist()[0][0:2]

def get_global_pose(robot_start_pose, init_odom, current_odom):
    ####################
    ## start location ##
    ####################
    #robot_start_pose = [x_start, y_start, alpha_start]

    ##########
    ## odom ##
    ##########
    #init_odom = [x_init_odom, y_init_odom]
    #current_odom = [x_current_odom, y_current_odom]

    # calculate odom
    local_odom_change = list(numpy.array(current_odom) - numpy.array(init_odom)) #current_odom - init_odom

    # convert from local to global
    global_pose = local2global(local_odom_change, robot_start_pose)

    return global_pose

if __name__ == "__main__":
    ###########################################
    ### coordinate transformation test case ###
    ###########################################
    global_pose = local2global([0,2],[0,2,math.pi/2])
    print global2local(global_pose, [0,2,math.pi/2])


