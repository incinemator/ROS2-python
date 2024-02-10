from robot_control_class import RobotControl
import time


class Maze_solver:

    def __init__(self, lspeed, t):
        self.robotcontrol = RobotControl()
        self.front_distance = 0
        self.distance = []
        self.angle_range = []
        self.linear_time = t
        self.linear_speed = lspeed

    # Check the robot's distance from front, left and right walls and print them on screen 
    def check_wall_distance(self):
        for i in self.angle_range:
            self.distance[i] = self.robotcontrol.get_laser(i)
        
    # Move the robot forward
    def walk(self):
        self.robotcontrol.move_straight()

    # Turn the robot 90 degrees to the riht
    def turn(self):
        self.robotcontrol.rotate(self.angles)
    
    
    # Re-center the robot
    # This method checks the difference between left and right wall distance.
    # If the difference is greater than 0.1 it attempts to re-center the robot
    # and checks if it is in a corner.
    # self.distance[0] is distance from the front
    def recenter(self):
        if not abs(self.distance[360] - self.distance[0]) < 0.1:
            # Robot is closer to the left
            if self.distance[360] > self.distance[0] and self.distance[360] < 1:
                self.turn()
                self.robotcontrol.move_straight_time("forward", self.linear_speed, self.linear_time)
                self.check_wall_distance()
            
            # Robot is in a left corner
            elif self.distance[360] > self.distance[0] and self.distance[360] > 1:
                self.robotcontrol.move_straight_time("backward", self.linear_speed,self.linear_time)
                self.turn()
                self.robotcontrol.move_straight_time("forward", self.linear_speed, self.linear_time)
                self.check_wall_distance()

            # Robot is closer to the right
            elif self.distance[0] > self.distance[360] and self.distance[0] < 1:
                self.turn()
                self.robotcontrol.move_straight_time("forward", self.linear_speed, self.linear_time)
                self.check_wall_distance()

            # Robot is in a right corner
            elif self.distance[0] > self.distance[360] and self.distance[0] > 1:
                self.robotcontrol.move_straight_time("backward", self.linear_speed, self.linear_time)
                self.turn()
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
