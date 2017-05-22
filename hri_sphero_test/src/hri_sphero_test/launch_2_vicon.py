#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import argparse
import random
from nav_msgs.msg import Odometry
import time
import math
import numpy
import ViconTrackerPoseHandler as vt



a = vt.ViconTrackerPoseHandler(None, None, "",51017, "Sphero2") 
print "Sphero2"
print a.getPose()
time.sleep(5)
print a.getPose()

b = vt.ViconTrackerPoseHandler(None, None, "",51018, "Sphero3")
print "Sphero3"
print b.getPose()
time.sleep(5)
print b.getPose()

c = vt.ViconTrackerPoseHandler(None, None, "",51020, "SpheroRGW")
print "SpheroRGW"
print c.getPose()
time.sleep(5)
print c.getPose()
