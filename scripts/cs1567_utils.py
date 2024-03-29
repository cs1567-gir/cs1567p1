#!/usr/bin/python
import math
import datetime
import rospy

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
        self.start_time = datetime.datetime.now()
        self.last_update = self.start_time # need current time
        self.elapsed_time = 0.0
        # updated by member functions
        self.drive_speed = 0.0
        self.turn_speed = 0.0
        self.ccw_turns = 0
        self.cw_turns = 0
        self.heading_correction = 0.0
        # set by node that instantiates the class
        self.command = None
        self.send_command = None
        # updated during sensors callback
        self.wheel_drop = 0
        
    def reset_class(self):
        self.pos_x = 0
        self.pos_y = 0
        self.orient_z = 0.0
        self.orient_w = 0.0
        self.heading = 0.0
        self.total_distance = 0.0
        self.start_time = datetime.datetime.now()
        self.last_update = self.start_time
        self.elapsed_time = 0.0

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
        if angle > 2*math.pi:
            angle -= 2*math.pi
        # print 'calculated angle: ', angle
        return angle

    def calculate_heading_correction(self):
        return (self.cw_turns - self.ccw_turns)*0.01

    def set_speeds(self, drive, turn):
        self.command.linear.x = drive
        self.command.angular.z = turn
        self.send_command(self.command)

    def stop_all_motion(self):
        self.set_speeds(0.0, 0.0)

    # move to given coordinates
    # TODO: copy logic from MoveXYOdometry (project0)
    def move_to(self, x, y):
        # logic from MoveXYOdometry
        drive_speed = 0.0
        turn_speed = 0.0
        x_error = x - self.pos_x
        y_error = y - self.pos_y
        distance = math.sqrt(x_error**2 + y_error**2)
        while(distance >= 0.02):
            
            x_error = x - self.pos_x
            y_error = y - self.pos_y
            distance = math.sqrt(x_error**2 + y_error**2)

            if distance < 0.25:
                drive_speed = 0.2
            else:
                drive_speed = 0.4

            # calculate target heading from coordinates
            target_heading = math.asin(y_error/distance)
            
            # compensate for domain and range issues of asin and acos
            if x_error < 0.0:
                target_heading = math.acos(x_error/distance)
                if y_error < 0.0:
                    target_heading = 2*math.pi - target_heading
            
            # constrain: 0 <= target_heading <= 2pi
            if target_heading < 0:
                target_heading += 2*math.pi
            if target_heading > 2*math.pi:
                target_heading -= 2*math.pi
            #print "target heading: ", target_heading
            # calculate how far we are from the target
            #target_heading += self.elapsed_time/21.0
            heading_error = target_heading - self.heading

            # constrain: -pi <= heading_error <= pi
            if heading_error > math.pi:
                heading_error -= 2*math.pi
            if heading_error < -math.pi:
                heading_error += 2*math.pi

            # scale error by distance to prevent increase in correction as distance decreases
            if distance < 0.5:
                heading_error = heading_error * 2 * distance
     
            # set turn speed
            if abs(heading_error) > 0.003:
                if abs(heading_error) > math.pi / 70:
                    drive_speed = 0.0
                if abs(heading_error) < 0.4:
                    turn_speed = math.copysign(0.2, heading_error)
                else:
                    turn_speed = heading_error * 0.5
            else:
                turn_speed = 0.0
            self.set_speeds(drive_speed, turn_speed)
        self.stop_all_motion()
        return 1

    # move forward a given distance
    def move_distance_OLD(self,dist):
        initial_distance = self.total_distance
        while (self.total_distance - initial_distance) < (0.98 * dist):
            compensated_dist = self.total_distance - initial_distance
            if dist - compensated_dist < 0.2:
                drive_speed = 0.25 
            else:
                drive_speed = 0.4
            self.set_speeds(drive_speed, 0.0)
        self.stop_all_motion()
        return 1

    def move_distance(self, dist, heading):
        drive_speed = 0.0
        turn_speed = 0.0
        initial_distance = self.total_distance
        while (self.total_distance - initial_distance) < (0.98 * dist):
            heading_error = heading - self.heading
            if heading_error < -math.pi:
                heading_error += 2*math.pi
            if heading_error > math.pi:
                heading_error -= 2*math.pi
            compensated_dist = self.total_distance - initial_distance
            if dist - compensated_dist < 0.2:
                drive_speed = 0.25
            else:
                drive_speed = 0.4
            if abs(heading_error) > 0.003:
                if abs(heading_error) < 0.3:
                    turn_speed = math.copysign(0.2, heading_error)
                else:
                    turn_speed = heading_error
            self.set_speeds(drive_speed, turn_speed)
        self.stop_all_motion()
        return 1

    # turn to specified angle relative to current heading
    # TODO: check this logic
    def turn_to_relative(self, theta):
        target_heading = self.heading + theta
        # target_heading += self.elapsed_time/21.0
        # ensure that: 0 <= target_heading <= 2pi
        if target_heading > 2*math.pi:
            target_heading -= 2*math.pi
        if target_heading < 0:
            target_heading += 2*math.pi

        error = target_heading - self.heading
        while(abs(error) > 0.003):
            if abs(error) < 2:
                self.set_speeds(0.0, math.copysign(0.2, error))
            else:
                self.set_speeds(0.0, error*0.1)
            error = target_heading - self.heading
            # ensure that: -pi <= error <= pi
            if error > math.pi:
                error -= 2*math.pi
            if error < -math.pi:
                error += 2*math.pi
        self.stop_all_motion()
        return 1

    # TODO: define this logic
    def turn_to_absolute(self, theta):
        # theta += self.heading_correction
        error = theta - self.heading
        if error > math.pi:
            error -= 2*math.pi
        if error < -math.pi:
            error += 2*math.pi
        if error < 0:
            self.cw_turns += 1
        if error > 0:
            self.ccw_turns += 1
        print "starting turn..."
        # define acceptible range for angle
        while(abs(error) > 0.003):
            if abs(error) < 0.4:
                self.set_speeds(0.0, math.copysign(0.4, error))
            else:
                self.set_speeds(0.0, error)
            error = theta - self.heading
            # ensure that: -pi < error < pi
            if error > math.pi:
                error -= 2*math.pi
            if error < -math.pi:
                error += 2*math.pi
        self.stop_all_motion()
        #print "ccw turns: ", self.ccw_turns, "cw turns: ", self.cw_turns
        return 1

    # drive an arc described by a radius and an angle
    def move_arc(self, radius, theta):
        turn_speed = 0.0
        initial_distance = self.total_distance
        initial_heading = self.heading
        arc_length = radius * theta # some calculation
        while(self.total_distance - initial_distance < abs(arc_length)):
            turn_speed = 0.0
            # calculate desired angle based on distance travelled
            target_heading = (self.total_distance - initial_distance)/radius
            target_heading = math.copysign(target_heading, arc_length)
            # offset by starting angle
            target_heading += initial_heading
            # ensure that 0 <= target_heading <= 2*pi
            if target_heading < 0:
                target_heading += 2*math.pi
            if target_heading > 2*math.pi:
                target_heading -= 2*math.pi

            error = target_heading - self.heading
            if error < -math.pi:
                error += 2*math.pi
            if error > math.pi:
                error -= 2*math.pi
            # correct angle
            if(abs(error) > 0.008):
                if abs(error) < 0.05:
                    turn_speed = math.copysign(0.3, error)  
                else:
                    turn_speed = error*5
            self.set_speeds(0.2, turn_speed)
            print "relative distance: ", self.total_distance - initial_distance
            print "error: ", error
        self.stop_all_motion()
        return 1

