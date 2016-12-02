#!/usr/bin/env python

import pygame
from pygame.locals import *
from sys import exit
from bluepy import btle
import math
import struct
import BB8_driver

import rospy

import math
import sys
import tf
import PyKDL

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from std_msgs.msg import ColorRGBA, Float32, Bool




name = 'bb8sphero'
rospy.init_node(name)
bb8 = BB8_driver.Sphero()
is_connected = 0




def parse_data_strm(data):
    odom_pub = rospy.Publisher(name + '/odom', Odometry, queue_size=1)
    imu_pub = rospy.Publisher(name + '/imu', Imu, queue_size=1)

    ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]


    ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                             0, 1e-3, 0, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e3]

    #if is_connected:
    now = rospy.Time.now()
    imu = Imu(header=rospy.Header(frame_id="imu_link"))
    imu.header.stamp = now
    imu.orientation.x = data["QUATERNION_Q0"]
    imu.orientation.y = data["QUATERNION_Q1"]
    imu.orientation.z = data["QUATERNION_Q2"]
    imu.orientation.w = data["QUATERNION_Q3"]
    imu.linear_acceleration.x = data["ACCEL_X_FILTERED"]/4096.0*9.8
    imu.linear_acceleration.y = data["ACCEL_Y_FILTERED"]/4096.0*9.8
    imu.linear_acceleration.z = data["ACCEL_Z_FILTERED"]/4096.0*9.8
    imu.angular_velocity.x = data["GYRO_X_FILTERED"]*10*math.pi/180
    imu.angular_velocity.y = data["GYRO_Y_FILTERED"]*10*math.pi/180
    imu.angular_velocity.z = data["GYRO_Z_FILTERED"]*10*math.pi/180

    imu_pub.publish(imu)

    odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_footprint')
    odom.header.stamp = now
    odom.pose.pose = Pose(Point(data["ODOM_X"]/100.0,data["ODOM_Y"]/100.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
    odom.twist.twist = Twist(Vector3(data["VELOCITY_X"]/1000.0, 0, 0), Vector3(0, 0, data["GYRO_Z_FILTERED"]*10.0*math.pi/180.0))
    odom.pose.covariance =ODOM_POSE_COVARIANCE                
    odom.twist.covariance =ODOM_TWIST_COVARIANCE

    odom_pub.publish(odom)
    rospy.loginfo(odom.pose.pose)


def start():
    global is_connected
    if is_connected==0:
        is_connected = bb8.connect()
    rospy.loginfo('BB8 connection status: {0}!'.format(is_connected))
    bb8.run()
    bb8.set_filtered_data_strm(int(400/50), 1 , 0, True)
    bb8.add_async_callback(BB8_driver.IDCODE['DATA_STRM'], parse_data_strm)
    bb8.set_rgb_led(0, 0, 0, 0, False)
    bb8.set_rotation_rate(1, False)
    #bb8.start() 

# Initializes Pygame & sets screen size (Width x Height)
pygame.init()
screen = pygame.display.set_mode((256, 275), 0, 32)
pygame.display.set_caption("BB8 Drive")

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
    if speed > 0:
        bb8.roll(speed, heading, 1, False)
    else:
        bb8.roll(0, lastHeading, 0, False)

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

    
    if joystick.get_button(6):
        bb8.roll(0, heading, 0, False)

    if joystick.get_button(4) == 1:
        bb8.set_heading(0, False)

    if joystick.get_button(5) == 1:
        bb8.set_back_led(255, False)

    if joystick.get_button(5) == 0:
        bb8.set_back_led(0, False)


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
    start()
    bb8.start()
    while not rospy.is_shutdown():
        clock.tick(10)
        
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
        axis_x = joystick.get_axis(0)
        axis_y = joystick.get_axis(1)



        draw_axis(screen, x, 0, axis_x, axis_y, axis_size)
        x += axis_size
        
        raw_data = bb8.run()
        #raw_data = 3
        print raw_data
        pygame.display.update()
