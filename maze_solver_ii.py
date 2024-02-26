from robot_control_class import RobotControl
from statistics import mean
import time


class Maze_solver:

    def __init__(self, lspeed, t):
        self.robotcontrol = RobotControl()
        self.single_distance = 0
        self.lidar_sector = [0.0] * 720 
        self.lidar_full = [0.0] * 720
        self.linear_time = t
        self.linear_speed = lspeed
        self.average_right = 0
        self.average_left = 0
        self.average_sector = 0
        # Angle variables
        self.rotate_angle = 0
        self.lidar_index = 0
        self.n_start = 0
        self.n_end = 0


    # Get a sector of LIDAR measuerments    
    def get_lidar_sector(self, n_start, n_end):
        for i in range(n_start, n_end):
            self.lidar_sector[i] = self.lidar_full[i]
        return self.lidar_sector 


    # Check the robot's distance from a single angle
    def check_distance_single(self):
        self.single_distance = self.robotcontrol.get_laser(self.lidar_index)
        
    # Move the robot forward
    def walk(self):
        self.robotcontrol.move_straight()

    # Turn the robot
    def turn(self):
        self.robotcontrol.rotate(self.rotate_angle)

            

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

            # Check if robot is near the exit
            
            self.get_lidar_sector(0, 180)
            self.average_sector = mean(self.lidar_sector)
            if self.average_sector > 10:
                break
            

            if self.single_distance < 0.8:
                print("Distance= ", self.single_distance)
                #self.robotcontrol.move_straight_time("backward", self.linear_speed,self.linear_time)
                self.lidar_full = self.robotcontrol.get_laser_full()
                self.get_lidar_sector(0, 359)
                self.average_right = mean(self.lidar_sector)
                print("Average distance to the right: ", self.average_right)
                self.get_lidar_sector(360, 719)
                self.average_left = mean(self.lidar_sector)
                print("Average distance to the left: ", self.average_left)

                if self.average_left < self.average_right:
                    self.rotate_angle = -90
                    self.turn()
                    self.robotcontrol.move_straight_time("forward",self.linear_speed,self.linear_time)
                    self.check_distance_single()
                else:
                    self.rotate_angle = 90
                    self.turn()
                    self.robotcontrol.move_straight_time("forward",self.linear_speed,self.linear_time)
                    self.check_distance_single()
            

        
        print("Found the exit, heading out")
        self.robotcontrol.move_straight()


ms1 = Maze_solver(0.2, 1)
# ms1.check_wall_distance()
ms1.solve()
