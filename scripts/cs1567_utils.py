#!/usr/bin/python
import math
import datetime

class KobukiRobot():
    def __init__(self):
        # updated during odometry callback
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.orient_z = 0.0
        self.orient_w = 0.0
        self.heading = 0.0
        self.velocity = 0.0
        self.omega = 0.0
        self.total_distance = 0.0
        self.last_update = datetime.datetime.now() # need current time
        # updated by member functions
        self.drive_speed = 0.0
        self.turn_speed = 0.0
        # set by node that instantiates the class
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
    # TODO: copy logic from MoveXYOdometry (project0)
    def move_to(self, x, y):
        # logic from MoveXYOdometry
        return 1

    # move forward a given distance
    def move_distance(self,dist):
        current_distance = self.total_distance
        while self.total_distance - current_distance < dist:
            self.set_speeds(0.3, 0.0)
        self.stop_all_motion()
        return 1

    # turn to specified angle relative to current heading
    # TODO: check this logic
    def turn_to_relative(self, theta):
        target_theta = theta - self.theta
        error = theta
        while(abs(error) > 0.001):
            if abs(error) < 2:
                self.set_speeds(0.0, math.copysign(0.2, error))
            else:
                self.set_speeds(0.0, error*0.1)
            error = target_theta - self.theta
        self.stop_all_motion()
        return 1

    # TODO: define this logic
    def turn_to_absolute(self, theta):
        error = theta - self.theta
        # define acceptible range for angle
        while(abs(error) > 0.001):
            if abs(error) < 2:
                self.set_speeds(0.0, math.copysign(0.2, error))
            else:
                self.set_speeds(0.0, error * 0.1)
            error = theta - self.theta
        self.stop_all_motion()
        return 1

    # drive an arc described by a radius and an angle
    def move_arc(self, radius, theta):
        current_distance = self.total_distance
        arc_length = radius * theta # some calculation
        while(self.total_distance - current_distance < arc_length):
            self.set_speeds(drive_speed, turn_speed)
            rospy.sleep(0.1) # sleep time can change
        self.stop_all_motion
        return 1

