import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 saniyede bir yayın

        self.angle_min = -math.pi / 4      # -45 derece
        self.angle_max = math.pi / 4       # +45 derece
        self.angle_increment = math.pi / 180  # 1 derece aralık
        self.range_min = 0.1
        self.range_max = 10.0

    def timer_callback(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)

        scan.ranges = []

        for i in range(num_readings):
            angle = self.angle_min + i * self.angle_increment
            # Ön 10 derece aralığında engel (mesafe 0.5m), diğer yerlerde 2.0m
            if -math.radians(5) <= angle <= math.radians(5):
                scan.ranges.append(0.5)
            else:
                scan.ranges.append(2.0)

        self.publisher_.publish(scan)
        self.get_logger().info('Published fake scan data with obstacle')

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

