#!/usr/bin/python
import argparse
import ast
import rospy
import tf
import copy

from nav_msgs.msg import Odometry, Path
import geometry_msgs.msg
import std_msgs.msg

import get_global_info

VELOCITY_COEFFICIENT = 3.0/4.0/100

class GlobalPoseObj(object):
    def __init__(self, args):
        # init odom
        self._current_odom = None
        self._init_odom = None

        ### JUST FOR TESTING #####
        #self._current_odom = [-1,0] # test
        #self._init_odom = [0,0] # test

        # get robot start pose
        self._robot_start_pose = ast.literal_eval(args.robot_start_pose)
        self._robot_frame = args.robot_frame

        #### PUBLISHERS #####
        # pose publisher
        self._pose_pub = rospy.Publisher(args.publish_pose_topic, geometry_msgs.msg.Pose, queue_size=10, latch=True)
        self._rate = rospy.Rate(30) # set publish rate

        # posestamped publisher
        #self._posestamped_pub = rospy.Publisher(args.publish_pose_topic, geometry_msgs.msg.PoseStamped, queue_size=10, latch=True)

        # publish heading (don't need this. Auto correct of heading by Sphero)
        #heading_pub = rospy.Publisher('set_heading', std_msgs.msg.Float32, queue_size=10, latch=True)
        #heading_pub.publish(0)

        # initialize posestampe msg
        self._posestamped_msg = geometry_msgs.msg.PoseStamped()
        self._posestamped_msg.header.frame_id = '/world'
        # initialize pose
        self._pose_msg = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0,0,0), \
                                          geometry_msgs.msg.Quaternion(0,0,0,1))
        self._posestamped_msg.pose = self._pose_msg

        self._br = tf.TransformBroadcaster()     # setup broadcaster

        # setup path
        self._path_msg = Path()
        self._path_msg.header.frame_id = '/world'
        self._path_pub = rospy.Publisher('path', Path, queue_size=10, latch=True) #args.robot_frame+'_path'

        # odom from velocity pub
        self._odom_from_velocity_pub = rospy.Publisher('odom_from_velocity', Odometry, queue_size=10, latch=True)
        self._odom_from_velocity_msg = Odometry()

        ### SUBSCRIBERS ####
        try:
            odom_msg = rospy.wait_for_message(args.odom_topic, Odometry, timeout=2.0)
        except:
            rospy.logwarn('{0}: Cannot subscribe to odometry from robot. Calulating our own.'.format(rospy.get_name()))
            odom_msg = None
        rospy.loginfo('{0}: odom_msgs: {1}'.format(rospy.get_name(), odom_msg))

        if not odom_msg: # we cannot get odom msgs
            # subscribe to velocity command
            rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, callback=self.extrapolate_odom_from_velocity)
            self._prev_cmd_time = rospy.Time.now().to_sec()
        else:
            # subscribe odom info
            rospy.Subscriber(args.odom_topic, Odometry, callback=self.save_odometry)


    def save_odometry(data):
        # save initial
        if self._current_odom is None:
            self._init_odom = [data.pose.pose.position.x, data.pose.pose.position.y]
            rospy.loginfo('init odom: {0}'.format(self._init_odom))

        self._current_odom = [data.pose.pose.position.x, data.pose.pose.position.y]
        rospy.loginfo('current odom: {0}'.format(self._current_odom))

        # forward odom
        self._odom_from_velocity_pub.publish(data)

    def extrapolate_odom_from_velocity(self, data):
        self._odom_from_velocity_msg.header.stamp = rospy.Time.now()
        self._odom_from_velocity_msg.header.frame_id = 'world'
        self._odom_from_velocity_msg.child_frame_id = self._robot_frame

        #set the velocity
        self._odom_from_velocity_msg.twist.twist = data

        #set the position
        if self._current_odom is None:
            self._init_odom = [0,0]
            self._current_odom = [0,0]
        else:
            time = self._odom_from_velocity_msg.header.stamp.to_sec() - self._prev_cmd_time
            if not time > 5.0: # !!!! if too long, that probably means we stopped in the middle
                self._current_odom[0] += data.linear.x*VELOCITY_COEFFICIENT*time
                self._current_odom[1] += data.linear.y*VELOCITY_COEFFICIENT*time
                rospy.loginfo('self._current_odom: {0}'.format(self._current_odom))

        # save data for calculation later
        self._prev_cmd_vel = data
        self._prev_cmd_time = self._odom_from_velocity_msg.header.stamp.to_sec()

        self._odom_from_velocity_msg.pose.pose.position.x = self._current_odom[0]
        self._odom_from_velocity_msg.pose.pose.position.y = self._current_odom[1]

        # pub odom
        self._odom_from_velocity_pub.publish(self._odom_from_velocity_msg)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Start Sphero Node")
    parser.add_argument('--robot_start_pose', type=str, help='Start pose of Sphero: [x_start, y_start, alpha_start]', nargs='?', const='[0,0,0]', default='[0,0,0]')
    parser.add_argument('--odom_topic', type=str, help='Specify odom topic of Sphero', nargs='?', const='odom', default='odom')
    parser.add_argument('--publish_pose_topic', type=str, help='Specify publish topic of Sphero pose', nargs='?', const='global_pose', default='global_pose')
    parser.add_argument('--robot_frame', type=str, help='name of robot frame', nargs='?', const='sphero', default='sphero')

    args, unknown = parser.parse_known_args()
    rospy.init_node(args.publish_pose_topic.lower().replace('-','_').replace('/','_'))

    # set up global pose obj
    global_pose_obj = GlobalPoseObj(args)

    prev_time = rospy.Time.now()
    #global_pose = [0,0] # test
    while not rospy.is_shutdown():
        ####################
        ## start location ##
        ####################
        #robot_start_pose = [x_start, y_start, alpha_start]

        ##########
        ## odom ##
        ##########
        #init_odom = [x_init_odom, y_init_odom]
        #current_odom = [x_current_odom, y_current_odom]

        ##########
        # return #
        ##########
        # global pose = [x_global, y_global]
        if global_pose_obj._current_odom and global_pose_obj._init_odom:
            global_pose = get_global_info.get_global_pose(global_pose_obj._robot_start_pose, \
                global_pose_obj._init_odom, global_pose_obj._current_odom)
        else: # assume we are staying in place for now
            global_pose = global_pose_obj._robot_start_pose[0:2]

        #global_pose[0] += 0.001 # test
        #global_pose[1] += 0.001 # test

        # Pose msg
        global_pose_obj._pose_msg.position.x = global_pose[0]
        global_pose_obj._pose_msg.position.y = global_pose[1]
        global_pose_obj._pose_pub.publish(global_pose_obj._pose_msg)

        # posestamped_msg
        global_pose_obj._posestamped_msg.header.stamp = rospy.Time.now()
        global_pose_obj._posestamped_msg.pose.position.x = global_pose[0]
        global_pose_obj._posestamped_msg.pose.position.y = global_pose[1]
        #global_pose_obj._posestamped_pub.publish(global_pose_obj._posestamped_msg)

        # update robot transform in rviz
        global_pose_obj._br.sendTransform((global_pose[0], global_pose[1], 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                args.robot_frame, "world")

        # update robot path in rviz
        if rospy.Time.now() - prev_time > rospy.Duration(0.5):
            global_pose_obj._path_msg.poses.append(copy.deepcopy(global_pose_obj._posestamped_msg))
            global_pose_obj._path_pub.publish(global_pose_obj._path_msg)
            prev_time = rospy.Time.now()
            #rospy.loginfo('We came here!' + str(global_pose_obj._path_msg))

        #rest
        global_pose_obj._rate.sleep()
