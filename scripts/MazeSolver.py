#!/usr/bin/python
import rospy
from cs1567p1.srv import *
from std_srvs.srv import * 
import math

LEFT = 0
RIGHT = 1
UP = 2
DOWN = 3

ROWS = 5
COLS = 5

class MazeNode():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.visited = False
        self.adjacency = [UP, RIGHT, DOWN, LEFT]

MazeGrid = [[]]
# assume all walls are OPEN
# build grid with no walls


make_maze_service = None
print_maze_service = None
get_wall_service = None
get_odom_service = None
constant_command_service = None

def init_grid():
    global MazeGrid
    MazeGrid = [[MazeNode(x, y) for x in xrange(ROWS)] for y in xrange(COLS)]

def move_forward():
    return 1

def turn_90_degrees():
    return 1

def solve_maze():
    global MazeGrid
    # stack
    move_stack = []
    x = 0
    y = 0
    while x != 5 and y != 5:
        current_node = x, y
        # get current orientation
        # check first based on that orientation
        if RIGHT in MazeGrid[x][y].adjacency:
            # turn to wall
            # check wall
            if get_wall_service(x, y, RIGHT):
                MazeGrid[x][y].adjacency.remove(RIGHT))
        if DOWN in MazeGrid[x][y].adjacency:
            # check wall
        if LEFT in MazeGrid[x][y].adjacency:
            # check wall
        if UP in MazeGrid[x][y].adjacency:
            # check wall
        # pick open direction
        	# turn to adjacency[0]
        # mark current node as visited       
        # put current node on stack
        # move to next node
    # if we are stuck, move back up the stack until we reach a node with an opening

    while True:
        data = get_odom_service(0)
        print data
        rospy.sleep(0.5)
    return 1

def initialize_commands():
    init_grid()
    rospy.init_node('mazesolvernode', anonymous=True)
    rospy.wait_for_service('make_maze')
    rospy.wait_for_service('print_maze')
    rospy.wait_for_service('get_wall')
    rospy.wait_for_service('get_odom')
#    rospy.wait_for_service('constant_command')

    global make_maze_service, print_maze_service, get_odom_service, get_wall_service
    global constant_command_service

    make_maze_service = rospy.ServiceProxy('make_maze', MakeNewMaze)
    print_maze_service = rospy.ServiceProxy('print_maze', Empty)
    get_wall_service = rospy.ServiceProxy('get_wall', GetMazeWall)
    get_odom_service = rospy.ServiceProxy('get_odom', GetOdometry)
#    constant_command_service = rospy.ServiceProxy('constant_command', ConstantCommand)

    make_maze_service(5,5)
    print_maze_service()
    solve_maze()

     
if __name__ == "__main__":   
    try: 
        initialize_commands()
    except rospy.ROSInterruptException: pass

