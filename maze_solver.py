from robot_control_class import RobotControl
import time


class Maze_solver:

    def __init__(self, lspeed, t):
        self.robotcontrol = RobotControl()
        self.front_distance = 0
        self.right_diagonal = 0
        self.left_diagonal = 0
        self.back_distance = 0
        self.right_distance = 0
        self.left_distance = 0
        self.linear_speed = lspeed
        self.right_angle = 90
        self.right_angle_low = 45
        self.right_angle_slight = 20
        self.left_angle = -90
        self.left_angle_low = -45
        self.left_angle_slight = -20
        self.linear_time = t

    # Check the robot's distance from front, left and right walls and print them on screen 
    def check_wall_distance(self):
        self.front_distance = self.robotcontrol.get_laser(360)
        self.right_diagonal = self.robotcontrol.get_laser(180)
        self.left_diagonal = self.robotcontrol.get_laser(539)
        self.left_distance = self.robotcontrol.get_laser(719)
        self.right_distance = self.robotcontrol.get_laser(0)
        print("Distance from front wall: ", self.front_distance)
        print("Distance from left wall: ", self.left_distance)
        print("Distance from right wall: ", self.right_distance)

    # Move the robot forward
    def walk(self):
        self.robotcontrol.move_straight()

    # Turn the robot 90 degrees to the riht
    def turn_right_ortho(self):
        self.robotcontrol.rotate(self.right_angle)
    
    # Turn the robot 45 degrees to the right
    def turn_right(self):
        self.robotcontrol.rotate(self.right_angle_low)

    # Turn the robot 20 degrees to the right

    def turn_slight_right(self):
        self.robotcontrol.rotate(self.right_angle_slight)

    # Turn the robot 90 degrees to the left
    def turn_left_ortho(self):
        self.robotcontrol.rotate(self.left_angle)

    # Turn the robot 45 degrees to the left
    def turn_left(self):
        self.robotcontrol.rotate(self.left_angle_low)
    
    # Turn the robot 20 degrees to the right
    def turn_slight_left(self):
        self.robotcontrol.rotate(self.left_angle_slight)
    
    # Re-center the robot
    # This method checks the difference between left and right wall distance.
    # If the difference is greater than 0.1 it attempts to re-center the robot
    # and checks if it is in a corner.
    def recenter(self):
        if not abs(self.right_distance - self.left_distance) < 0.1:
            # Robot is closer to the left
            if self.right_distance > self.left_distance and self.right_distance < 1:
                self.turn_slight_right()
                self.robotcontrol.move_straight_time("forward", self.linear_speed, self.linear_time)
                self.check_wall_distance()
            
            # Robot is in a left corner
            elif self.right_distance > self.left_distance and self.right_distance > 1:
                self.robotcontrol.move_straight_time("backward", self.linear_speed,self.linear_time)
                self.turn_right()
                self.robotcontrol.move_straight_time("forward", self.linear_speed, self.linear_time)
                self.check_wall_distance()

            # Robot is closer to the right
            elif self.left_distance > self.right_distance and self.left_distance < 1:
                self.turn_slight_left()
                self.robotcontrol.move_straight_time("forward", self.linear_speed, self.linear_time)
                self.check_wall_distance()

            # Robot is in a right corner
            elif self.left_distance > self.right_distance and self.left_distance > 1:
                self.robotcontrol.move_straight_time("backward", self.linear_speed, self.linear_time)
                self.turn_left()
                self.robotcontrol.move_straight_time("forward", self.linear_speed, self.linear_time)
        else:
            self.robotcontrol.move_straight_time("forward", self.linear_speed, self.linear_time)
            self.check_wall_distance()

    # The maze solving method: While the distance from the front wall is less than 10 m, the robot
    # moves forward step by step, checking wall distances in each step. 
    def solve(self):

        while self.front_distance < 10:

            self.robotcontrol.move_straight_time(
                "forward", self.linear_speed, self.linear_time)

            self.check_wall_distance()
            self.recenter()

            if self.front_distance < 0.8:

                if self.left_diagonal - self.right_diagonal < 0:
                    if self.left_distance < 0.5:
                        print("Left corner, turning right")
                        self.robotcontrol.move_straight_time("backward", self.linear_speed,self.linear_time)
                        self.turn_right_ortho()
                        self.robotcontrol.move_straight_time("forward",self.linear_speed,self.linear_time)
                        self.check_wall_distance()
                    elif self.right_distance < 0.5:
                        print("Right corner, turning left")
                        self.robotcontrol.move_straight_time("backward", self.linear_speed,self.linear_time)
                        self.turn_left_ortho()
                        self.robotcontrol.move_straight_time("forward",self.linear_speed,self.linear_time)
                        self.check_wall_distance()
                    else:
                        print("Too close to the wall on the right, movig away to the left")
                        self.robotcontrol.move_straight_time("backward", self.linear_speed,self.linear_time)
                        self.turn_left()
                        self.robotcontrol.move_straight_time("forward",self.linear_speed,self.linear_time)  
                        self.check_wall_distance()   

                elif self.right_diagonal - self.left_diagonal < 0:
                    if self.left_distance < 0.5:
                        print("Left corner, turning right")
                        self.robotcontrol.move_straight_time("backward", self.linear_speed,self.linear_time)
                        self.turn_right_ortho()
                        self.robotcontrol.move_straight_time("forward",self.linear_speed,self.linear_time)
                        self.check_wall_distance()
                    elif self.right_distance < 0.5:
                        print("Right corner, turning left")
                        self.robotcontrol.move_straight_time("backward", self.linear_speed,self.linear_time)
                        self.turn_left_ortho()
                        self.robotcontrol.move_straight_time("forward",self.linear_speed,self.linear_time)
                        self.check_wall_distance()
                    else:
                        print("Too close to the wall on the left, movig away to the right")
                        self.robotcontrol.move_straight_time("backward", self.linear_speed,self.linear_time)
                        self.turn_right()
                        self.robotcontrol.move_straight_time("forward",self.linear_speed,self.linear_time)
                        self.check_wall_distance()

            else:
                self.recenter()
                self.robotcontrol.move_straight_time(
                    "forward", self.linear_speed, self.linear_time)
        
        print("Found the exit, heading out")
        self.robotcontrol.move_straight()


ms1 = Maze_solver(0.1, 1)
# ms1.check_wall_distance()
ms1.solve()
