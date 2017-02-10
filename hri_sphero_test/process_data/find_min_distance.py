import rospy
import itertools
import math
from collections import OrderedDict
import argparse
import rosbag

import geometry_msgs.msg

CONTROLLED_AGENT = 'sphero_ggw'

class DataProcessing(object):
    def __init__(self):
        self.sphero_list = ['sphero_rgw', 'sphero_wrb', 'sphero_yoo', 'sphero_wpp', CONTROLLED_AGENT]
        self.pose_dict = OrderedDict([(name,[0.0,0.0]) for name in self.sphero_list])

        # for calculating completion time
        self.start_time = None
        self.end_time = None
        self.prev_CONTROLLED_AGENT_pose = None

        # for saving distance
        #self.dist[this_sphero][another_sphero]
        self.dist = OrderedDict()
        for this_sphero in self.sphero_list:
            self.dist[this_sphero] = OrderedDict()
            for another_sphero in self.sphero_list:
                if this_sphero != another_sphero:
                    self.dist[this_sphero][another_sphero] = 100.0

            # subscribers
            #rospy.loginfo("/"+this_sphero+'/global_pose')
            rospy.Subscriber("/"+this_sphero+'/global_pose', geometry_msgs.msg.Pose,
                callback=self.calculate_min_distance, callback_args=(this_sphero,))

        #rospy.on_shutdown(self.print_result)

    def print_result(self):
        # print data
        print("#### Min distance ####")
        print("{0:15s}\t{1:10s}\t{2:10s}\t{3:10s}\t{4:10s}\t{5:10s}".format(\
            'Current\Another', *self.sphero_list))
        for idx, this_sphero in enumerate(self.sphero_list):
            dist_list = self.dist[this_sphero].values()
            dist_list.insert(idx, 9999.0)
            print("{0:15s}\t{1:10.4f}\t{2:10.4f}\t{3:10.4f}\t{4:10.4f}\t{5:10.4f}".format(\
                this_sphero, *dist_list))

        print("\n#### Time to Complete ####")
        if self.end_time and self.start_time:
            print(str(self.end_time - self.start_time)+'\n')
        elif not self.end_time:
            print ("NO END TIME!\n")
        elif not self.start_time:
            print ("NO END TIME!\n")

    def calculate_min_distance(self, data, args):
        self.pose_dict[args[0]] = [data.position.x, data.position.y]
        #rospy.loginfo('got data')
        #rospy.loginfo(args)
        #rospy.loginfo(data)

        # calculate distance only when everyone is ready
        if all(itertools.chain(self.pose_dict.values())):

            if args[0] == CONTROLLED_AGENT:
                if not self.prev_CONTROLLED_AGENT_pose:
                    self.prev_CONTROLLED_AGENT_pose = self.pose_dict[args[0]]
                elif not self.start_time and \
                math.sqrt((self.prev_CONTROLLED_AGENT_pose[0]- self.pose_dict[args[0]][0])**2 +
                (self.prev_CONTROLLED_AGENT_pose[1]- self.pose_dict[args[0]][1])**2) > 0.10:
                    if len(args) > 1:
                        self.start_time = args[1]
                    else:
                        self.start_time = rospy.Time.now().to_sec()
                elif self.start_time and not self.end_time and \
                math.sqrt((3.735 - self.pose_dict[args[0]][0])**2 +
                (self.pose_dict[args[0]][1]- 1.825)**2) < 0.20:
                    if len(args) > 1:
                        self.end_time = args[1]
                    else:
                        self.end_time = rospy.Time.now().to_sec()

                # update pose
                self.prev_CONTROLLED_AGENT_pose = self.pose_dict[args[0]]

            for another_sphero, another_sphero_pose in self.pose_dict.iteritems():
                if another_sphero != args[0]:
                    dist = math.sqrt((data.position.x-another_sphero_pose[0])**2 +\
                                    (data.position.y-another_sphero_pose[1])**2)
                    self.dist[args[0]][another_sphero] = min(self.dist[args[0]][another_sphero], dist)
                    self.dist[another_sphero][args[0]] = min(self.dist[another_sphero][args[0]], dist)


class TestSubjectDataProcessing(object):
    # calculate eight and square
    # only for ggw
    def __init__(self):
        self._square_center = [1.5, 1.5]
        self._eight_points = [[1.0, 1.0], [1.0, 2.0]]
        self._min_square_dist = 9999.0
        self._max_square_dist = 0.0
        self._min_eight_dist = 9999.0
        self._max_eight_dist = 0.0

    def calculate_square_distance(self, data):
        pose = [data.transform.translation.x, data.transform.translation.y]
        dist = math.sqrt((self._square_center[0]- pose[0])**2 +
                         (self._square_center[1]- pose[1])**2)
        self._min_square_dist = min(self._min_square_dist, dist)
        self._max_square_dist = max(self._max_square_dist, dist)

    def calculate_eight_distance(self, data):
        pose = [data.transform.translation.x, data.transform.translation.y]

        dist_0 = math.sqrt((self._eight_points[0][0]- pose[0])**2 +
                         (self._eight_points[0][1]- pose[1])**2)
        dist_1 = math.sqrt((self._eight_points[1][0]- pose[0])**2 +
                         (self._eight_points[1][1]- pose[1])**2)
        if dist_0 > dist_1:
            dist = dist_1
        else:
            dist = dist_0

        self._min_eight_dist = min(self._min_eight_dist, dist)
        self._max_eight_dist = max(self._max_eight_dist, dist)

    def print_square_result(self):
        print("------ Square Result ------")
        print("{0:15s}\t{1:15s}".format('Min Distance', 'Max Distance'))
        print("{0:10.4f}\t{1:10.4f}\n".format(self._min_square_dist, self._max_square_dist))


    def print_eight_result(self):
        print("------ Eight Result ------")
        print("{0:15s}\t{1:15s}".format('Min Distance', 'Max Distance'))
        print("{0:10.4f}\t{1:10.4f}\n".format(self._min_eight_dist, self._max_eight_dist))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process data")
    parser.add_argument('--participant_number', type=int, \
        help='Participant number', nargs='?', const=1, default=1)
    parser.add_argument('--check_goal', type=str, \
        help='look at messages', nargs='?', const='False', default='False')

    args, unknown = parser.parse_known_args()
    rospy.init_node('find_min_distance')

    a = DataProcessing()

    if args.check_goal == 'True':
        bag_type_list = ['joystick']
        #topics = ['/'+CONTROLLED_AGENT+'/global_pose']
        topics = ['/'+'sphero_yoo'+'/global_pose']
        #bag_type_list = ['helmet']
        #topics = ['/'+CONTROLLED_AGENT+'/global_pose']
    else:
        bag_type_list = ['observation', 'joystick', 'helmet']
        topics = ['/'+sphero_name+'/global_pose' for sphero_name in a.sphero_list]

    print "~~~~~~ Participant {0} ~~~~~~~~".format(args.participant_number)
    eight_bag = rosbag.Bag('/home/catherine/bagfiles/Participant'+\
        str(args.participant_number)+'/'+'eight'+'.bag')
    square_bag = rosbag.Bag('/home/catherine/bagfiles/Participant'+\
        str(args.participant_number)+'/'+'square'+'.bag')
    b = TestSubjectDataProcessing()
    for topic, msg, t in eight_bag.read_messages(topics=['/vicon/sphero_ggw/body']):
        b.calculate_eight_distance(msg)
    for topic, msg, t in square_bag.read_messages(topics=['/vicon/sphero_ggw/body']):
        b.calculate_square_distance(msg)

    b.print_eight_result()
    b.print_square_result()


    for bag_type in bag_type_list:
        print "------ {bag_type} ------".format(bag_type=bag_type)
        bag = rosbag.Bag('/home/catherine/bagfiles/Participant'+\
            str(args.participant_number)+'/'+bag_type+'.bag')
        a = DataProcessing()

        if args.check_goal == 'True': start = None

        for topic, msg, t in bag.read_messages(topics=topics):
            if args.check_goal == 'True' and not start:
                start = t.to_sec()
                print msg, t.to_sec()
            a.calculate_min_distance(msg, [topic.replace('/global_pose','').replace('/',''), \
            t.to_sec()])

        a.print_result()

        if args.check_goal == 'True': print msg, t.to_sec(), t.to_sec()-start
        bag.close()

    #rospy.spin()


