import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import sys
import math

class EyeDrawer(Node):
    def __init__(self):
        super().__init__('eye_drawer')
        self.get_logger().info('Eye Drawer Node Initialized')
        self.init_pygame()
        self.expression = 'neutral'  # 默认为中立表情
        self.direction = 'neutral'  # 默认为中立方向
        self.target_pupil_position = (0, 0)  # 目标瞳孔位置
        self.current_pupil_position = (0, 0)  # 当前瞳孔位置
        self.pupil_speed = 0.5 # 瞳孔移动速度
        self.subscription = self.create_subscription(
            String,
            'eye_expression',
            self.expression_callback,
            10
        )

    def expression_callback(self, msg):
        parts = msg.data.split('_')  # 分割表情和方向部分
        if len(parts) == 2:
            self.expression = parts[0]
            self.direction = parts[1]
            self.target_pupil_position = self.get_pupil_offset(self.direction)  # 更新目标瞳孔位置
            self.get_logger().info(f"Received expression: {self.expression}, direction: {self.direction}")
        else:
            self.get_logger().warn(f"Invalid message format: {msg.data}. Expected 'expression_direction'.")

    def init_pygame(self):
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))  # 设置窗口大小
        pygame.display.set_caption('Draw Eyes')  # 窗口标题
        self.clock = pygame.time.Clock()

    def draw_eye(self, x, y): 
        # 绘制眼睛
        pygame.draw.ellipse(self.screen, (255, 255, 255), (x - 50, y - 40, 100, 80))  # 外圆（眼白）
        pygame.draw.arc(self.screen, (0, 0, 0), (x - 50, y - 40, 100, 80), 6.18, 3.14, width=3)  # 黑色眼眶
        pygame.draw.arc(self.screen, (255, 0, 0), (x - 50, y - 40, 100, 80), 3.14, 6.18, width=2)  # 黑色眼眶

        # 调整虹膜、瞳孔和高光点的位置
        adjusted_x, adjusted_y = self.current_pupil_position

        pygame.draw.ellipse(self.screen, (100, 100, 255), (x + adjusted_x - 20, y + adjusted_y - 20, 40, 40))  # 蓝色内圆（虹膜）
        if self.expression == 'surprised':
            pygame.draw.ellipse(self.screen, (0, 0, 0), (x + adjusted_x - 10, y + adjusted_y - 10, 30, 30))  # 瞳孔放大
        else:
            pygame.draw.ellipse(self.screen, (0, 0, 0), (x + adjusted_x - 10, y + adjusted_y - 10, 20, 20))  # 默认瞳孔
        pygame.draw.ellipse(self.screen, (255, 255, 255), (x + adjusted_x - 5, y + adjusted_y - 5, 8, 8))  # 白色高光点

        # 绘制眉毛
        if self.expression == 'angry':
            pygame.draw.arc(self.screen, (150, 75, 0), (x - 50, y - 80, 100, 50), 3.14, 6.28, width=6)  # 生气的眉毛
        else:
            pygame.draw.arc(self.screen, (150, 75, 0), (x - 50, y - 80, 100, 50), 6.28, 3.14, width=6)  # 中立的眉毛

    def draw_mouth(self, x, y):
        # 绘制嘴巴
        if self.expression == 'surprised':
            pygame.draw.ellipse(self.screen, (0, 0, 0), (x - 30, y + 20, 60, 30))  # 张大嘴巴
        elif self.expression == 'angry':
            pygame.draw.arc(self.screen, (0, 0, 0), (x - 30, y, 60, 30), 6.28, 3.14, width=3)  # 嘴巴下翘
        else:
            pygame.draw.arc(self.screen, (0, 0, 0), (x - 30, y, 60, 30), 3.14, 6.28, width=3)  # 默认嘴巴

    def get_pupil_offset(self, direction):
        # 根据方向返回瞳孔的偏移量
        pupil_offset = {
            'left': (-20, 0),   # 向左看
            'right': (20, 0),   # 向右看
            'up': (0, -10),     # 向上看
            'down': (0, 10),    # 向下看
            'neutral': (0, 0)   # 中立（默认）
        }
        return pupil_offset.get(direction, (0, 0))

    def update_pupil_position(self):
        # 更新瞳孔位置，使其平滑过渡到目标位置
        dx = self.target_pupil_position[0] - self.current_pupil_position[0]
        dy = self.target_pupil_position[1] - self.current_pupil_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        if distance > self.pupil_speed:
            step_x = dx / distance * self.pupil_speed
            step_y = dy / distance * self.pupil_speed
            self.current_pupil_position = (self.current_pupil_position[0] + step_x, self.current_pupil_position[1] + step_y)
        else:
            self.current_pupil_position = self.target_pupil_position

    def run_pygame(self):
        # Pygame 事件循环
        running = True
        while running:
            for event in pygame.event.get(): 
                if event.type == pygame.QUIT:
                    running = False
                    self.get_logger().info('Pygame window closed. Shutting down ROS 2 node.')
                    self.destroy_node()
                    rclpy.shutdown()
                    pygame.quit()
                    sys.exit()

            self.screen.fill((255, 255, 255))  # 白色背景
            eye_x_left, eye_y_left = 130, 150
            eye_x_right, eye_y_right = 270, 150
            mouth_x, mouth_y = 200, 250  # 嘴巴的中心位置

            self.update_pupil_position()  # 更新瞳孔位置

            self.draw_eye(eye_x_left, eye_y_left)
            self.draw_eye(eye_x_right, eye_y_right)
            self.draw_mouth(mouth_x, mouth_y)
            pygame.display.flip()
            self.clock.tick(30)

            # 处理 ROS 2 消息
            rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    # ROS 2 初始化
    rclpy.init(args=args)
    node = EyeDrawer()
    node.run_pygame()  # 在主线程中运行 Pygame 事件循环
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()