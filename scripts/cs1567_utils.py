#!/usr/bin/python
import math

class KobukiRobot():
    def __init__(self):
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.orient_z = 0.0
        self.orient_w = 0.0
	self.theta = 0.0
        self.velocity = 0.0
        self.total_distance = 0.0
        self.drive_speed = 0.0
        self.turn_speed = 0.0
        self.command = None
        self.send_command = None

    # return yaw IN RADIANS with respect to the starting orientation
    # output will be from zero to 2*pi
    def calculate_heading(self):
        # calculations
        mag = math.sqrt(self.orient_w**2 + self.orient_z**2)
        w = self.orient_w / mag
        z = self.orient_z / mag
        angle = math.copysign(2*math.acos(w), z)
        if angle < 0:
            angle = angle + 2*math.pi
        return angle

    def set_speeds(self, drive, turn):
        self.command.linear.x = drive
        self.command.linear.z = turn
        self.send_command(self.command)

    def stop_all_motion(self):
        self.set_speeds(0.0, 0.0)

    # move to given coordinates
    def move_to(self, x, y):
        # logic from before
        return 1

    # move forward a given distance
    def move_distance(self,dist):
        current_distance = self.total_distance
        while self.total_distance - current_distance < dist:
            self.set_speeds(drive, 0.0)
        self.stop_all_motion()

    # turn to specified angle
    def turn_to(self, theta):
        current_theta = self.theta
        while
        return 1

    # drive an arc described by a radius and an angle
    def move_arc(self, radius, theta):
        current_distance = self.total_distance
        arc_length = 0 # some calculation
        while(self.total_distance - current_distance < ARC_LENGTH):
            self.set_speeds(drive_speed, turn_speed)
            rospy.sleep(0.1) # sleep time can change
        self.stop_all_motion
        return 0

