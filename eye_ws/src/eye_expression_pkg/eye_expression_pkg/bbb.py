import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image  # 确保导入了Image类
from rclpy.qos import qos_profile_sensor_data

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        
        # 创建图像订阅者
        self.image_sub = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10)
        self.image_sub  # prevent unused variable warning

        # 创建图像发布者
        self.image_pub = self.create_publisher(Image, 'processed_image', 10)

        # 读取静态图像
        self.image_path = '/home/zhao/图片/截图/微信图片_20250210222354.jpg'  # 替换为你的图像路径
        self.static_image = self.load_static_image()

    def load_static_image(self):
        image = cv2.imread(self.image_path)
        if image is None:
            self.get_logger().info('Failed to load image')
            return None
        return image

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info('cv_bridge exception: %s' % e)
            return

        # Process the image
        processed_image = self.process_image(cv_image)

        # Publish the processed image
        self.publish_image(processed_image)

        # Display the processed image
        cv2.imshow('Processed Image', processed_image)
        cv2.waitKey(3)

    def process_image(self, image):
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply thresholding
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours
        for contour in contours:
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)

        return image

    def publish_image(self, image):
        # Convert OpenCV image to ROS Image message
        try:
            image_message = self.bridge.cv2_to_imgmsg(image, "bgr8")
            image_message.header.stamp = self.get_clock().now().to_msg()
        except CvBridgeError as e:
            self.get_logger().info('cv_bridge exception: %s' % e)
            return

        # Publish the image
        self.image_pub.publish(image_message)
        self.get_logger().info('Published processed image')

    def publish_static_image(self):
        if self.static_image is not None:
            try:
                image_message = self.bridge.cv2_to_imgmsg(self.static_image, "bgr8")
                image_message.header.stamp = self.get_clock().now().to_msg()
            except CvBridgeError as e:
                self.get_logger().info('cv_bridge exception: %s' % e)
                return

            self.image_pub.publish(image_message)
            self.get_logger().info('Published static image')

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    timer_period = 1.0  # 发布频率
    image_processor.create_timer(timer_period, image_processor.publish_static_image)
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()