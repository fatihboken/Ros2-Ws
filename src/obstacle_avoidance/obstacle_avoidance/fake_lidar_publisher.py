import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.angle_min = -math.pi / 4
        self.angle_max = math.pi / 4
        self.angle_increment = math.pi / 180  # 1 derece
        self.range_max = 10.0

    def timer_callback(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.5
        scan.range_min = 0.0
        scan.range_max = self.range_max

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [2.0] * num_readings  # Sabit 2 metre mesafe

        self.publisher_.publish(scan)
        self.get_logger().info('Published fake scan data')

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
