#!/usr/bin/python
import rospy
from cs1567p1.srv import *
from std_srvs.srv import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
import sys
from cs1567_utils import KobukiState

last_state = KobukiState()
odom = None

def odometry_callback(data):
    # called when odometry is received
    global odom
    odom = data
    # should also update global state

def odometry_service(parameters):
    # return current and relevant odometry data
    resp = GetOdometryResponse(odom.pose.pose.position.x,
                               odom.pose.pose.position.y,
                               odom.pose.pose.orientation.z,
                               odom.pose.pose.orientation.w)
    return resp

def get_state(parameters):
    # return the overall state of the robot
    pass

def initialize():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('odometryservicenode', anonymous=True)
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())
    s1 = rospy.Service('get_odom', GetOdometry, odometry_service)
    rospy.spin()

if __name__ == "__main__":
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass
