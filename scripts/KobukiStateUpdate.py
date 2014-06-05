#!/usr/bin/python
import rospy
from cs1567p1.srv import *
from std_srvs.srv import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
import sys
from cs1567_utils import KobukiRobot
import math
import datetime

robot_state = None
constant_command_service = None

def odometry_callback(data):
    # called when odometry is received
    global robot_state

    current_time = datetime.datetime.now()

    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    current_w = data.pose.pose.orientation.w
    current_z = data.pose.pose.orientation.z

    old_heading = robot_state.heading

    delta_x = current_x - robot_state.pos_x
    delta_y = current_y - robot_state.pos_y
    dist_from_last = math.sqrt(delta_x**2 + delta_y**2)
    # should also update global state
    robot_state.pos_x = current_x
    robot_state.pos_y = current_y
    robot_state.orient_z = current_z
    robot_state.orient_w = current_w
    robot_state.heading = robot_state.calculate_heading()
    delta_t = ((float)(current_time.microsecond) - (float)(robot_state.last_update.microsecond)) / 1000000 # delta t in seconds
    robot_state.last_update = current_time
    current_velocity = dist_from_last / delta_t
    current_omega = (robot_state.heading - old_heading) / delta_t
    robot_state.velocity = current_velocity
    robot_state.omega = current_omega
    robot_state.total_distance += dist_from_last


def odometry_service(parameters):
    # return current and relevant odometry data
    resp = GetOdometryResponse(robot_state.pos_x,
                               robot_state.pos_y,
                               robot_state.orient_z,
                               robot_state.orient_w,
                               robot_state.heading)
    return resp

def move_service(parameters):
    if parameters.type == 'move_to':
        retval = robot_state.move_to(parameters.a, parameters.b) 
    elif parameters.type == 'move_distance':
        retval = robot_state.move_distance(parameters.a)
    elif parameters.type == 'move_arc':
        retval = robot_state.move_arc(parameters.a, parameters.b)
    return CS1567RobotMoveResponse(retval)

def turn_service(parameters):
    if parameters.type == 'relative':
        retval = robot_state.turn_to_relative(parameters.theta)
    elif parameters.type == 'absolute':
        retval = robot_state.turn_to_absolute(parameters.theta)
    return CS1567RobotTurnResponse(retval)

def stop_all_motion(parameters):
    retval = robot_state.stop_all_motion()
    return StopAllResponse(retval)

def get_state(parameters):
    # return the overall state of the robot
    pass

def initialize():
    global robot_state
    global constant_command_service
    rospy.wait_for_service('constant_command')
    robot_state = KobukiRobot()
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('odometryservicenode', anonymous=True)
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())
    s1 = rospy.Service('get_odom', GetOdometry, odometry_service)
    s2 = rospy.Service('cs1567_move', CS1567RobotMove, move_service)
    s3 = rospy.Service('cs1567_turn', CS1567RobotTurn, turn_service)
    s4 = rospy.Service('cs1567_stop_all_motion', StopAll, stop_all_motion)

    robot_state.send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
    robot_state.command = Twist()
    rospy.spin()

if __name__ == "__main__":
    try:
        initialize()
    except rospy.ROSInterruptException:
        pass
