import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DummyCameraSubscriber(Node):
    def __init__(self):
        super().__init__('dummy_camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # ROS Image mesajını OpenCV görüntüsüne çevir
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info('Received image frame')

        # Görüntüyü göster (OpenCV penceresi)
        cv2.imshow("Received Image", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DummyCameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

