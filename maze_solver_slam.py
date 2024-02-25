import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class TurtlebotSLAM:
    def __init__(self):
        rospy.init_node('turtlebot_slam', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)

        self.laser_data = None
        self.odom_data = None

        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.threshold_distance = 0.8
        self.threshold_angle = 15  # in degrees

    def laser_callback(self, data):
        self.laser_data = data

    def odom_callback(self, data):
        self.odom_data = data

    def get_yaw_angle(self):
        orientation_q = self.odom_data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return math.degrees(yaw)

    def move_forward(self):
        twist_cmd = Twist()
        twist_cmd.linear.x = self.linear_speed
        self.cmd_vel_pub.publish(twist_cmd)

    def rotate(self, angular_speed):
        twist_cmd = Twist()
        twist_cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist_cmd)

    def stop(self):
        twist_cmd = Twist()
        self.cmd_vel_pub.publish(twist_cmd)

    def find_exit(self):
        while not rospy.is_shutdown():
            if self.laser_data is not None and self.odom_data is not None:
                front_distance = min(self.laser_data.ranges[:10] + self.laser_data.ranges[-10:])
                current_yaw = self.get_yaw_angle()

                if front_distance < self.threshold_distance:
                    self.stop()
                    rospy.sleep(1)

                    if abs(current_yaw) < self.threshold_angle:
                        self.move_forward()
                        rospy.sleep(2)
                    elif current_yaw > 0:
                        self.rotate(-self.angular_speed)
                        rospy.sleep(1)
                    else:
                        self.rotate(self.angular_speed)
                        rospy.sleep(1)
                else:
                    self.move_forward()

            self.rate.sleep()

if __name__ == '__main__':
    try:
        turtlebot = TurtlebotSLAM()
        turtlebot.find_exit()
    except rospy.ROSInterruptException:
        pass
