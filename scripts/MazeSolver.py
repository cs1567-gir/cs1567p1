#!/usr/bin/python
import rospy
from cs1567p1.srv import *
from std_srvs.srv import * 
import math

LEFT = 0
RIGHT = 1
UP = 2
DOWN = 3

MazeGrid = [[]]

make_maze_service = None
print_maze_service = None
get_wall_service = None
get_odom_service = None
constant_command_service = None
move_service = None
turn_service = None
stop_service = None

def move_forward():
    return 1

def turn_90_degrees():
    return 1

def solve_maze():
   #move_service('move_distance', 1.0, 0.0)
   #turn_service('relative', math.pi/2)
   move_service('move_arc', 0.5, math.pi)
   rospy.sleep(0.25)
   print "moving second arc"
   move_service('move_arc', 1.0, -2*math.pi)
   return 1
   # while True:
   #     data = get_odom_service(0)
   #     print data
   #     rospy.sleep(0.5)

def initialize_commands():
    rospy.init_node('mazesolvernode', anonymous=True)
    rospy.wait_for_service('make_maze')
    rospy.wait_for_service('print_maze')
    rospy.wait_for_service('get_wall')
    rospy.wait_for_service('get_odom')
    rospy.wait_for_service('cs1567_move')
    rospy.wait_for_service('cs1567_turn')
    rospy.wait_for_service('cs1567_stop_all_motion')
#    rospy.wait_for_service('constant_command')

    global make_maze_service, print_maze_service, get_odom_service, get_wall_service
    global constant_command_service
    global move_service
    global turn_service
    global stop_service

    make_maze_service = rospy.ServiceProxy('make_maze', MakeNewMaze)
    print_maze_service = rospy.ServiceProxy('print_maze', Empty)
    get_wall_service = rospy.ServiceProxy('get_wall', GetMazeWall)
    get_odom_service = rospy.ServiceProxy('get_odom', GetOdometry)
#    constant_command_service = rospy.ServiceProxy('constant_command', ConstantCommand)
    move_service = rospy.ServiceProxy('cs1567_move', CS1567RobotMove)
    turn_service = rospy.ServiceProxy('cs1567_turn', CS1567RobotTurn)
    stop_service = rospy.ServiceProxy('cs1567_stop_all_motion', StopAll)

    make_maze_service(5,5)
    print_maze_service()
    solve_maze()

     
if __name__ == "__main__":   
    try: 
        initialize_commands()
    except rospy.ROSInterruptException: pass

