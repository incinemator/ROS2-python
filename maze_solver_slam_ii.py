import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener
from tf2_ros import TransformStamped
from tf2_ros.buffer import Buffer
from math import degrees
import math

class TurtlebotSLAM(Node):
    def __init__(self):
        super().__init__('turtlebot_slam')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription_odom  # prevent unused variable warning
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.transform_buffer = Buffer()
        self.listener = TransformListener(self.transform_buffer, self)
        self.angular_speed = 0.5
        self.linear_speed = 0.2
        self.threshold_distance = 0.8
        self.threshold_angle = 15  # in degrees

    def laser_callback(self, msg):
        front_ranges = msg.ranges[:10] + msg.ranges[-10:]
        self.front_distance = min(front_ranges)

    def odom_callback(self, msg):
        self.odom_data = msg

    def get_yaw_angle(self):
        try:
            trans = self.transform_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            orientation = trans.transform.rotation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(orientation_list)
            return math.degrees(yaw)
        except Exception as e:
            self.get_logger().error('Failed to get transform: {}'.format(e))
            return None

    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        self.publisher.publish(msg)

    def rotate(self, angular_speed):
        msg = Twist()
        msg.angular.z = angular_speed
        self.publisher.publish(msg)

    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)

    def find_exit(self):
        while rclpy.ok():
            try:
                current_yaw = self.get_yaw_angle()
                if self.front_distance is not None:
                    if self.front_distance < self.threshold_distance:
                        self.stop()
                        self.get_logger().info('Obstacle detected!')
                        rclpy.sleep(1)

                        if abs(current_yaw) < self.threshold_angle:
                            self.move_forward()
                            rclpy.sleep(2)
                        elif current_yaw > 0:
                            self.rotate(-self.angular_speed)
                            rclpy.sleep(1)
                        else:
                            self.rotate(self.angular_speed)
                            rclpy.sleep(1)
                    else:
                        self.move_forward()

            except Exception as e:
                self.get_logger().error('Error in find_exit: {}'.format(e))
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    try:
        turtlebot = TurtlebotSLAM()
        turtlebot.find_exit()
    except KeyboardInterrupt:
        pass
    finally:
        turtlebot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
