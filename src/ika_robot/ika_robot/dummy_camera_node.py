import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class DummyCameraNode(Node):
    def __init__(self):
        super().__init__('dummy_camera')
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.bridge = CvBridge()
        self.counter = 0

    def timer_callback(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        x = (self.counter * 10) % 640
        cv2.rectangle(img, (x, 200), (x+50, 250), (0, 255, 0), -1)  # yeşil kare

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published dummy image frame {self.counter}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = DummyCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

