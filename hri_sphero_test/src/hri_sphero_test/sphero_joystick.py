#!/usr/bin/env python

import pygame
from pygame.locals import *
from sys import exit
from bluepy import btle
import math
import struct
#import BB8_driver
#from sphero_driver import sphero_driver
#from sphero import SpheroNode

import rospy

import math
import numpy as np
import sys
import tf
import PyKDL
import dynamic_reconfigure.server
import argparse

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from std_msgs.msg import ColorRGBA, Float32, Bool




# parser = argparse.ArgumentParser(description="Start Sphero Node")
# parser.add_argument('--target_name', type=str, help='Specify name of Sphero', nargs='?', \
#                         const='', default='')
# parser.add_argument('--target_addr', type=str, help='Specify address of Sphero', nargs='?', const='', default='')
# parser.add_argument('--port', type=int, help='Specify port of Sphero', nargs='?', const=1, default=1)

# args, unknown = parser.parse_known_args()
# if args.target_name:
#     rospy.init_node(args.target_name.lower().replace('-','_')+'_joystick')
# else:
#     rospy.init_node('Sphero')
#rospy.loginfo(args)
#rospy.loginfo('Sphero name: {0}'.format(args.target_name))

#sph = SpheroNode(args.target_name, args.target_addr, args.port)
#rospy.loginfo('Connecting to Sphero name: {0} ...'.format(args.target_name))
#sph.start()
#rospy.loginfo('Sphero name: {0} connected!'.format(args.target_name))

#sph.run()
# bb8.set_filtered_data_strm(int(400/50), 1 , 0, True)
# bb8.add_async_callback(BB8_driver.IDCODE['DATA_STRM'], parse_data_strm)

#sph.set_rgb_led(255, 0, 0, 0, False)
#sph.set_rotation_rate(1, False)

rospy.init_node('sphero_joystick')

cmd_vel_pub = rospy.Publisher('/sphero_ggw/cmd_vel', Twist, queue_size = 1)
#cmd_vel_pub = rospy.Publisher('/sphero_ypw/cmd_vel', Twist, queue_size = 1)

def publish_cmd_vel(v, thet):
    # Convert speed and heading to cmd_vel and publish
    scale = 90
    #scale = 200
    vx = v*math.sin(math.pi*thet/180) # atan2 in cmd_vel has opposite order atan2(vx,vy) instead of atan2(y,x)
    vy = v*math.cos(math.pi*thet/180)
    eps = np.finfo(float).eps
    vx = vx*scale/(v+eps)
    vy = vy*scale/(v+eps)
    linear = Vector3(vx,vy,0)
    angular = Vector3(0,0,math.pi*thet/180)
    pub_msg = Twist(linear,angular)
    cmd_vel_pub.publish(pub_msg)
    # self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.x,msg.linear.y))*180/math.pi
    # self.cmd_speed = math.sqrt(math.pow(msg.linear.x,2)+math.pow(msg.linear.y,2))
    # self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)


# def start():
#     global is_connected
#     if is_connected==0:
#         is_connected = bb8.connect()
#     rospy.loginfo('BB8 connection status: {0}!'.format(is_connected))
#     bb8.run()
#     bb8.set_filtered_data_strm(int(400/50), 1 , 0, True)
#     bb8.add_async_callback(BB8_driver.IDCODE['DATA_STRM'], parse_data_strm)
#     bb8.set_rgb_led(0, 0, 0, 0, False)
#     bb8.set_rotation_rate(1, False)
#     #bb8.start() 

# Initializes Pygame & sets screen size (Width x Height)
pygame.init()
screen = pygame.display.set_mode((256, 275), 0, 32)
pygame.display.set_caption("Sphero Joystick Drive")

pygame.joystick.init()
pygame.joystick.Joystick(0).init()


clock = pygame.time.Clock()


def clamp(value, minValue, maxValue):
    if value > maxValue:
        return maxValue
    elif value < minValue:
        return minValue
    else:
        return value

def mapRange(value, inMin, inMax, outMin, outMax):
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def sendRollCommand(speed, heading):
    lastHeading = 0
    if heading > 0:
       lastHeading = heading
    # if speed > 0:
    #     #sph.roll(speed, heading, 1, False)
    # else:
    #     #sph.roll(0, lastHeading, 0, False)

    publish_cmd_vel(speed, heading)

def draw_axis(surface, x, y, axis_x, axis_y, size):
    line_col = (128, 128, 128)
    num_lines = 40
    step = size / float(num_lines)
    joystickCenterX = 128
    joystickCenterY = 128
    

    # Draws grid
    for n in range(num_lines):
        line_col = [(64, 64, 64), (89, 89, 89)][n & 1]
        pygame.draw.line(surface, line_col, (x + n * step, y), (x + n * step, y + size))
        pygame.draw.line(surface, line_col, (x, y + n * step), (x + size, y + n * step))

    pygame.draw.line(surface, (200, 200, 200), (x, y + size / 2), (x + size, y + size / 2))
    pygame.draw.line(surface, (200, 200, 200), (x + size / 2, y), (x + size / 2, y + size))

    # Constrains analog stick input into X & Y coordinates of screen size
    draw_x = int(x + (axis_x * size + size) / 2.)
    draw_y = int(y + (axis_y * size + size) / 2.)

    # Scales x_length & y_length to make sure coordinates are polar instead of elliptical
    x_length = draw_x - joystickCenterX
    y_length = joystickCenterY - draw_y

    if joystickCenterX > joystickCenterY:
        x_length *= joystickCenterY / joystickCenterX
    elif joystickCenterX < joystickCenterY:
        y_length *= joystickCenterX / joystickCenterY

    # Calculates joystick position distance from center
    if joystickCenterX > 0.0 and joystickCenterY > 0.0:
        joystickDistanceFromCenter = math.sqrt(x_length * x_length + y_length * y_length) / min(joystickCenterX, joystickCenterY)
        joystickDistanceFromCenter = clamp(joystickDistanceFromCenter, 0.0, 1.0)
    else:
        joystickDistanceFromCenter = 0.0

    # Calculate the angle
    joystickAngleDegrees = math.atan2(x_length, y_length)

    # Adjust for range between 0 and 2
    if joystickAngleDegrees < 0.0:
        joystickAngleDegrees += 2.0 * math.pi

    # Convert to degrees
    joystickAngleDegrees *= 180.0 / math.pi

    speed = int(mapRange(joystickDistanceFromCenter, 0.0, 1.0, 0, 255))
    heading = int(joystickAngleDegrees)

    sendRollCommand(speed, heading)

    
    # if joystick.get_button(6):
    #     sph.roll(0, heading, 0, False)

    # if joystick.get_button(4) == 1:
    #     sph.set_heading(0, False)

    # if joystick.get_button(5) == 1:
    #     sph.set_back_led(255, False)

    # if joystick.get_button(5) == 0:
    #     sph.set_back_led(0, False)


    # Displays the joystick X & Y coordinates to screen
    message = "X: {}  Y: {} Speed: {} Heading: {}".format(draw_x, draw_y, speed, heading)
    font = pygame.font.SysFont("arial", 15);
    text_surface = font.render(message, True, (255, 153, 0))
    screen.blit(text_surface, (1, 260))

    # Calculates joystick indicator and vector line
    draw_pos = (draw_x, draw_y)
    center_pos = (x + size / 2, y + size / 2)

    # Draws joystick indicator and vector line
    pygame.draw.line(surface, (114, 153, 0), center_pos, draw_pos, 2)
    pygame.draw.circle(surface, (134, 179, 0), draw_pos, 10)




if __name__ == '__main__':
    r = rospy.Rate(10.0)
    #start()
    #bb8.start()
    while not rospy.is_shutdown():
        #clock.tick(100)
        
        joystick = pygame.joystick.Joystick(0)

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                exit()

        if event.type == KEYDOWN:
            if event.key >= K_0 and event.key <= K_1:
                num = event.key - K_0


        # axis_size = min(256, 640 / (joystick.get_numaxes()/2))
        axis_size = min(256, 256)

        #    pygame.draw.rect(screen, (255, 255, 255), (0, 0, 256, 275))
        pygame.draw.rect(screen, (38, 38, 38), (0, 0, 256, 275))

        # Draw all the axes (analog sticks)
        x = 0
        axis_x = joystick.get_axis(2)
        axis_y = joystick.get_axis(3)



        draw_axis(screen, x, 0, axis_x, axis_y, axis_size)
        x += axis_size
        
        #raw_data = sph.run()
        #raw_data = 3
        #print raw_data
        r.sleep()
        pygame.display.update()
