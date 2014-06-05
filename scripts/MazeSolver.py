#!/usr/bin/python
import rospy
from cs1567p1.srv import *
from std_srvs.srv import * 
import math


LEFT = 0
RIGHT = 1
UP = 2
DOWN = 3

dir_names = ['LEFT', 'RIGHT', 'UP', 'DOWN']

class MazeNode():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.visited = False
        self.openings = [UP, RIGHT, DOWN, LEFT]

    # return the next available opening to drive through
    def get_next_opening(self):
        return self.openings[0]

    # remove an opening from the list (a.k.a. add a wall)
    def remove_opening(self, direction):
        self.openings.remove(direction)

    def swap_to_end(self, direction):
        self.openings.remove(direction)
        self.openings.append(direction)

MazeGrid = []
glo_dirs = [UP, RIGHT, DOWN, LEFT]

make_maze_service = None
print_maze_service = None
get_wall_service = None
get_odom_service = None
constant_command_service = None
move_service = None
turn_service = None
stop_service = None

def init_maze():
    global MazeGrid
    MazeGrid.append([MazeNode(0,0)])

def move_forward():
    return 1

def turn_90_degrees():
    return 1

def get_relative_coordinates(direction):
    if direction == LEFT:
        return -1, 0
    elif direction == RIGHT:
        return 1, 0
    elif direction == UP:
        return 0, -1
    else:
        return 0, 1

def get_angle(direction):
    if direction == LEFT:
        return math.pi/2
    elif direction == RIGHT:
        return 3*math.pi/2
    elif direction == UP:
        return 0.0
    else:
        return math.pi

def get_real_coordinates(x, y):
    return -0.5*y, -0.5*x

def opposite_direction(direction):
    if direction == UP:
        return DOWN
    elif direction == DOWN:
        return UP
    elif direction == LEFT:
        return RIGHT
    else:
        return LEFT

def print_matrix():
    output = ''
    for col in range(len(MazeGrid)):
        for row in range(len(MazeGrid[col])):
            output += ' N'
        output += '\n'
    print output

def solve_maze(target_x, target_y):
    global MazeGrid
    last_direction = -1
    move_stack = []
    x = 0
    y = 0
    while x != target_x or y != target_y:
        current_node = MazeGrid[x][y] # this is a MazeNode
        print "Position: ", current_node.x, ", ", current_node.y
        for direction in glo_dirs:
            if direction in current_node.openings and not current_node.visited and not (direction == last_direction):
                # turn to direction
                turn_service('absolute', get_angle(direction))
                # check for wall
                wall = get_wall_service(x, y, direction)
                #print "value of wall: ", wall
                if wall.wall == 1: #remove opening
                    current_node.remove_opening(direction)
                    print "wall at: ", dir_names[direction]
                else:
                    print "no wall at: ", dir_names[direction]
                relative_coords = get_relative_coordinates(direction)
                next_x = x + relative_coords[0]
                next_y = y + relative_coords[1]
                if (next_x >= 0) and (next_y >= 0): 
                    max_coord = max(next_x, next_y)
                    # if there is no row/col at that index
                    # print "Grid size: ", len(MazeGrid), "max_coord: ", max_coord
                    if len(MazeGrid) == max_coord:
                        #print "adding new row and column to grid"
                        # add new row and column to the matrix (maintain square)
                        MazeGrid.append([]) # adds new column
                        for row in range(len(MazeGrid) - 1):
                            MazeGrid[max_coord].append(MazeNode(max_coord, row))
                        for col in range(max_coord + 1):
                            #print "appending node at ", col, ", ", max_coord
                            MazeGrid[col].append(MazeNode(col, max_coord))
                        #print "new grid row size: ", len(MazeGrid)
                        #print "new grid column size: ", len(MazeGrid[1])
                        print_matrix()
                    if wall.wall == 1:
                        MazeGrid[next_x][next_y].remove_opening(opposite_direction(direction))
        direction = current_node.get_next_opening()
        last_direction = opposite_direction(direction)
        relative_coords = get_relative_coordinates(direction)
        next_x = x + relative_coords[0]
        next_y = y + relative_coords[1]

        MazeGrid[next_x][next_y].swap_to_end(opposite_direction(direction))
       
        # determine next coordinates from our direction
        real_coords = get_real_coordinates(next_x, next_y)
        move_service('move_to', real_coords[0], real_coords[1])
        x = next_x
        y = next_y
        current_node.visited = True

    return 1
   

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
    init_maze()
    solve_maze(4,4)

     
if __name__ == "__main__":   
    try: 
        initialize_commands()
    except rospy.ROSInterruptException: pass

