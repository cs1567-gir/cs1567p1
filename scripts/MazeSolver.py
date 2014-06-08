#!/usr/bin/python
import rospy
from cs1567p1.srv import *
from std_srvs.srv import * 
from kobuki_msgs.msg import *
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
        self.openings = [RIGHT, DOWN, LEFT, UP]

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
glo_dirs = [[LEFT, UP, RIGHT, DOWN],
            [RIGHT, UP, LEFT, DOWN],
            [UP, LEFT, DOWN, RIGHT],
            [DOWN, LEFT, UP, RIGHT]]


make_maze_service = None
print_maze_service = None
get_wall_service = None
get_odom_service = None
constant_command_service = None
move_service = None
turn_service = None
stop_service = None
reset_odometry_service = None


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

# add a new row and column to the matrix
def expand_matrix(new_index):
    MazeGrid.append([]) # adds new column
    for row in range(len(MazeGrid) - 1):
        MazeGrid[new_index].append(MazeNode(new_index, row))
    for col in range(new_index + 1):
        MazeGrid[col].append(MazeNode(col, new_index))
    return 1

def print_matrix():
    output = ''
    for col in range(len(MazeGrid)):
        for row in range(len(MazeGrid[col])):
            output += ' N'
        output += '\n'
    print output

def solve_maze(target_x, target_y):
    move = 0
    '''
    snd_pub = rospy.Publisher('/mobile_base/commands/sound', Sound)
    sound = Sound()
    sound.value = 6
    print sound
    snd_pub.publish(sound)
    '''
    use_xy = False
    global MazeGrid
    last_direction = -1
    move_stack = []
    x = 0
    y = 0
    current_direction = UP
    while x != target_x or y != target_y:
        move += 1
        current_node = MazeGrid[x][y] # this is a MazeNode
        print move, ". Position: ", current_node.x, ", ", current_node.y
        for direction in glo_dirs[current_direction]:
            if direction in current_node.openings and not current_node.visited and not (direction == last_direction):
                # turn to direction
                turn_service('absolute', get_angle(direction))
                rospy.sleep(0.25)
                if direction == UP:
                    reset_odometry_service()
                rospy.sleep(0.5)
                # check for wall
                wall = get_wall_service(x, y, direction)
                #print "value of wall: ", wall
                if wall.wall == 1: #remove opening
                    current_node.remove_opening(direction)
                    print "\t", dir_names[direction], ": WALL"
                else:
                    print "\t", dir_names[direction], ": NO WALL"
                relative_coords = get_relative_coordinates(direction)
                next_x = x + relative_coords[0]
                next_y = y + relative_coords[1]
                if (next_x >= 0) and (next_y >= 0): 
                    max_coord = max(next_x, next_y)
                    # if we haven't placed new nodes in the direction we are looking
                    if len(MazeGrid) == max_coord:
                        expand_matrix(max_coord)
                        print_matrix()
                    if wall.wall == 1:
                        MazeGrid[next_x][next_y].remove_opening(opposite_direction(direction))
        direction = current_node.get_next_opening()
        current_direction = direction
        last_direction = opposite_direction(direction)
        relative_coords = get_relative_coordinates(direction)
        next_x = x + relative_coords[0]
        next_y = y + relative_coords[1]

        MazeGrid[next_x][next_y].swap_to_end(opposite_direction(direction))
       
        # determine next coordinates from our direction
        real_coords = get_real_coordinates(next_x, next_y)
        rospy.sleep(0.25)
        turn_service('absolute', get_angle(direction))
        rospy.sleep(0.25)
        #if use_xy:
        #    move_service('move_to', real_coords[0], real_coords[1])
        #    use_xy = False
        #else:
        move_service('move_distance', 0.48, get_angle(direction))
        #    use_xy = True
        rospy.sleep(0.25)
        x = next_x
        y = next_y
        current_node.visited = True
    return 1
   

def celebrate(led1pub, led2pub, sndpub):
    count = 0
    color = 1
    snd_msg = Sound()
    snd_msg.value = 6
    led1_msg = Led()
    led2_msg = Led()
    try:
        while True:
            sndpub.publish(snd_msg)
            while count < 10:
                #flash leds at current color
                if count % 2 == 0:
                    led1_msg.value = color
                    led2_msg.value = 0
                else:
                    led1_msg.value = 0
                    led2_msg.value = color
                led1pub.publish(led1_msg)
                led2pub.publish(led2_msg)
                rospy.sleep(0.2)
                count += 1
                color += 1
                if color > 3:
                    color = 1
            count = 0
    except:
        led1_msg.value = 0
        led2_msg.value = 0
        led1pub.publish(led1_msg)
        led2pub.publish(led2_msg)
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
    global reset_odometry_service

    make_maze_service = rospy.ServiceProxy('make_maze', MakeNewMaze)
    print_maze_service = rospy.ServiceProxy('print_maze', Empty)
    get_wall_service = rospy.ServiceProxy('get_wall', GetMazeWall)
    get_odom_service = rospy.ServiceProxy('get_odom', GetOdometry)
#    constant_command_service = rospy.ServiceProxy('constant_command', ConstantCommand)
    move_service = rospy.ServiceProxy('cs1567_move', CS1567RobotMove)
    turn_service = rospy.ServiceProxy('cs1567_turn', CS1567RobotTurn)
    stop_service = rospy.ServiceProxy('cs1567_stop_all_motion', StopAll)
    reset_odometry_service = rospy.ServiceProxy('cs1567_reset_odometry', CSReset)

    snd_pub = rospy.Publisher('/mobile_base/commands/sound', Sound)
    led1_pub = rospy.Publisher('/mobile_base/commands/led1', Led)
    led2_pub = rospy.Publisher('/mobile_base/commands/led2', Led)

    make_maze_service(5,5)
    print_maze_service()
    init_maze()
    solve_maze(4,4)
    # celebrate(led1_pub, led2_pub, snd_pub)
     
if __name__ == "__main__":   
    try: 
        initialize_commands()
    except rospy.ROSInterruptException: pass

