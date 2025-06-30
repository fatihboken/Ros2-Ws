import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan',          # fake_lidar_publisher ile aynı topic
            self.lidar_callback,
            10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.threshold_distance = 1.0  # metre cinsinden engel mesafesi sınırı

    def lidar_callback(self, msg: LaserScan):
        # LIDAR verisinin ön kısmını (örneğin ortadaki 30 derece) alıyoruz
        center_index = len(msg.ranges) // 2
        window_size = 15  # ±15 okuma (~30 derece)
        front_angles = msg.ranges[center_index - window_size : center_index + window_size]

        # Geçersiz (0 veya sonsuz) değerleri filtrele
        valid_ranges = [dist for dist in front_angles if msg.range_min < dist < msg.range_max]

        if not valid_ranges:
            self.get_logger().info('No valid LIDAR data received')
            return

        min_distance = min(valid_ranges)

        twist = Twist()
        if min_distance < self.threshold_distance:
            self.get_logger().info(f'Obstacle detected at {min_distance:.2f} m! Turning right.')
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # sağa dönüş hızı
        else:
            self.get_logger().info(f'Path clear. Moving forward. Min distance: {min_distance:.2f} m')
            twist.linear.x = 0.2  # ileri hız
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

