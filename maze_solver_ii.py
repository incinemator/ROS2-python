from robot_control_class import RobotControl
from statistics import mean
import time


class Maze_solver:

    def __init__(self, lspeed, t):
        self.robotcontrol = RobotControl()
        self.single_distance = 0
        self.distance = [None] * 720
        self.linear_time = t
        self.linear_speed = lspeed
        self.average_right = 0
        self.average_left = 0
        # Angle variables
        self.rotate_angle = 0
        self.lidar_index = 0
        self.n_start = 0
        self.n_end = 719
        

    # Check the robot's distance from a single angle
    def check_distance_single(self):
        self.single_distance = self.robotcontrol.get_laser(self.lidar_index)


    # Check the robots distance from several angles
    def check_distance(self):
        for i in range(self.n_start, self.n_end):
            self.distance[i] = self.robotcontrol.get_laser(i)
        
    # Move the robot forward
    def walk(self):
        self.robotcontrol.move_straight()

    # Turn the robot
    def turn(self):
        self.robotcontrol.rotate(self.rotate_angle)

    # Calculate average distance
    def average_distance(self):

        average = mean(self.distance)

        return average
            

    # self.distance[0]: distance from the right
    # self.distance[719]: distance from the left 

    # The maze solving method: While the distance from the front wall is less than 10 m, the robot
    # moves forward step by step, checking wall distances in each step. 
    def solve(self):

        while self.single_distance < 10:

            self.robotcontrol.move_straight_time(
                "forward", self.linear_speed, self.linear_time)
            self.lidar_index = 360
            self.check_distance_single()
            

            if self.single_distance < 0.8:
                print("Distance= ", self.single_distance)
                self.robotcontrol.move_straight_time("backward", self.linear_speed,self.linear_time)
                self.n_start = 0
                self.n_start = 360
                self.check_distance()
                self.average_right = self.average_distance()
                self.n_start = 360
                self.n_end = 719
                self.average_left = self.average_distance()

                if self.average_left < self.average_right:
                    self.rotate_angle = -90
                    self.turn()
                    self.robotcontrol.move_straight_time("forward",self.linear_speed,self.linear_time)
                    self.check_distance_single()

        
        print("Found the exit, heading out")
        self.robotcontrol.move_straight()


ms1 = Maze_solver(0.1, 1)
# ms1.check_wall_distance()
ms1.solve()
