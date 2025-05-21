import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image

class EyeExpressionController(Node):
    def __init__(self):
        super().__init__('eye_expression_controller')
        self.bridge = CvBridge()
        self.image_path = '/home/zhao/图片/截图/微信图片_20250210222354.jpg'  # 替换为您的图像路径
        self.original_image = self.load_image()
        self.eye_center = (100, 100)  # 假设眼睛的初始位置已知
        self.pupil_radius = 10  # 瞳孔半径

    def load_image(self):
        image = cv2.imread(self.image_path)
        if image is None:
            self.get_logger().info('Failed to load image')
        return image

    def adjust_eye_position(self, horizontal, vertical):
        # 加载原始图像
        image = self.original_image.copy()
        
        # 计算新位置
        new_eye_center = (self.eye_center[0] + horizontal, self.eye_center[1] + vertical)
        
        # 确保新位置在图像范围内
        new_eye_center = (max(0, min(new_eye_center[0], image.shape[1]-1)),
                         max(0, min(new_eye_center[1], image.shape[0]-1)))
        
        # 在图像上绘制新的眼睛位置
        cv2.circle(image, new_eye_center, self.pupil_radius, (0, 255, 0), 2)
        
        # 显示图像
        cv2.imshow('Adjusted Eye Position', image)
        cv2.waitKey(1)

    def display_image(self):
        while rclpy.ok():
            cv2.imshow('Original Image', self.original_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = EyeExpressionController()
    
    # 模拟接收到的指令
    horizontal = 10  # 水平移动距离
    vertical = 5  # 垂直移动距离
    
    node.adjust_eye_position(horizontal, vertical)
    
    node.display_image()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()