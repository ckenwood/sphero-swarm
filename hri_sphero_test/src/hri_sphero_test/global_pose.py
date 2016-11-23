#!/usr/bin/python
import argparse
import ast
import rospy
import global_info

from nav_msgs.msg import Odometry
import geometry_msgs.msg
import std_msgs.msg

current_odom = None
init_odom = None

current_odom = [0,0]
init_odom = [0,0]

def save_odometry(data):
    global current_odom

    # save initial
    if current_odom is None:
        global init_odom
        init_odom = [data.pose.pose.position.x, data.pose.pose.position.y]
        rospy.loginfo('init odom: {0}'.format(init_odom))

    current_odom = [data.pose.pose.position.x, data.pose.pose.position.y]
    rospy.loginfo('current odom: {0}'.format(current_odom))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Start Sphero Node")
    parser.add_argument('--robot_start_pose', type=str, help='Start pose of Sphero: [x_start, y_start, alpha_start]', nargs='?', const='[0,0,0]', default='[0,0,0]')
    parser.add_argument('--odom_topic', type=str, help='Specify odom topic of Sphero', nargs='?', const='odom', default='odom')
    parser.add_argument('--publish_pose_topic', type=str, help='Specify publish topic of Sphero pose', nargs='?', const='sphero_pose', default='sphero_pose')

    args, unknown = parser.parse_known_args()
    rospy.init_node(args.publish_pose_topic.lower().replace('-','_').replace('/','_'))

    # get robot start pose
    robot_start_pose = ast.literal_eval(args.robot_start_pose)

    # subscribe odom info
    rospy.Subscriber(args.odom_topic, Odometry, callback=save_odometry)

    # pose publisher
    pose_pub = rospy.Publisher(args.publish_pose_topic, geometry_msgs.msg.Pose, queue_size=10, latch=True)
    rate = rospy.Rate(30) # set publish rate

    # publish heading
    heading_pub = rospy.Publisher('set_heading', std_msgs.msg.Float32, queue_size=10, latch=True)
    heading_pub.publish(0)

    pose_msg = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(0,0,0), \
                                      geometry_msgs.msg.Quaternion(0,0,0,1))
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
        if current_odom and init_odom:
            global_pose = global_info.get_global_pose(robot_start_pose, init_odom, current_odom)
            pose_msg.position.x = global_pose[0]
            pose_msg.position.y = global_pose[1]
            pose_pub.publish(pose_msg)

        #rest
        rate.sleep()
