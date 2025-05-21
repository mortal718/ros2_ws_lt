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
        self.pupil_speed = 3.0  # 瞳孔移动速度
        self.target_mouth_position = 0  # 目标嘴巴位置
        self.current_mouth_position = 0  # 当前嘴巴位置
        self.mouth_speed = 1  # 嘴巴变化速度
        self.target_eyebrow_position = 0  # 目标眉毛位置
        self.current_eyebrow_position = 0  # 当前眉毛位置
        self.eyebrow_speed = 1  # 眉毛变化速度

        # 眨眼相关变量
        self.blink_state = 'open'  # 眨眼状态：'open' 或 'closed'
        self.blink_progress = 0  # 眨眼进度（0 到 1）
        self.blink_speed = 10  # 眨眼速度
        self.blink_duration = 1.5  # 眨眼持续时间（秒）
        self.is_blinking = False  # 是否正在眨眼

        self.subscription = self.create_subscription(
            String,
            'eye_expression',
            self.expression_callback,
            10
        )

    def expression_callback(self, msg):
        parts = msg.data.split('_')  # 分割消息部分
        if len(parts) >= 2:  # 确保至少有表情和方向两个部分
            self.expression = parts[0]
            self.direction = parts[1]
            self.num = len(parts)
            self.target_pupil_position = self.get_pupil_offset(self.direction)  # 更新目标瞳孔位置
            self.target_mouth_position = self.get_mouth_offset(self.expression)  # 更新目标嘴巴位置
            self.target_eyebrow_position = self.get_eyebrow_offset(self.expression)  # 更新目标眉毛位置

            # 检查是否包含眨眼指令
            if 'blink' in parts:
                self.is_blinking = True
                self.blink_state = 'open'  # 从睁开开始
                self.blink_progress = 0  # 重置眨眼进度
        else:
            self.get_logger().warn(f"Invalid message format: {msg.data}. Expected format: 'expression_direction' or 'blink'")

    def init_pygame(self):
        pygame.init()
        self.screen = pygame.display.set_mode((2060, 1545))  # 设置窗口大小
        pygame.display.set_caption('')  # 窗口标题
        self.clock = pygame.time.Clock()

    def draw_eye(self, x, y):
        # 根据眨眼进度动态调整上眼皮位置
        if self.is_blinking:
            eyelid_position = self.blink_progress * 160  # 动态调整上眼皮位置
        else:
            eyelid_position = 0

        # 增大眼白的尺寸
        pygame.draw.ellipse(self.screen, (255, 255, 255), (x - 300, y - 240 + eyelid_position, 600, 480 - eyelid_position * 2))  # 外圆（眼白）

        # 增大虹膜的尺寸
        adjusted_x, adjusted_y = self.current_pupil_position
        pygame.draw.ellipse(self.screen, (100, 100, 255), (x + adjusted_x - 120, y + adjusted_y - 120 + eyelid_position, 240, 240 - eyelid_position * 2))  # 蓝色内圆（虹膜）

        # 增大瞳孔的尺寸
        pupil_color = (0, 0, 0)
        if self.expression == 'surprised':
            pygame.draw.ellipse(self.screen, pupil_color, (x + adjusted_x - 75, y + adjusted_y - 75 + eyelid_position, 150, 150 - eyelid_position * 2))  # 瞳孔放大
        else:
            pygame.draw.ellipse(self.screen, pupil_color, (x + adjusted_x - 60, y + adjusted_y - 60 + eyelid_position, 120, 120 - eyelid_position * 2))  # 默认瞳孔

        # 增大高光点的尺寸
        pygame.draw.ellipse(self.screen, (255, 255, 255), (x + adjusted_x - 30, y + adjusted_y - 30 + eyelid_position, 60, 60 - eyelid_position))  # 白色高光点

        # 调整眼线（上眼皮）
        pygame.draw.arc(self.screen, (0, 0, 0), (x - 300, y - 240 + eyelid_position, 600, 480 - eyelid_position * 2), 6.18, 3.14, width=18)
        # 调整眼线（下眼皮）
        pygame.draw.arc(self.screen, (255, 0, 0), (x - 300, y - 240, 600, 480), 3.14, 6.18, width=12)

        # 增大眉毛的尺寸
        adjusted_eyebrow_y = self.current_eyebrow_position
        if self.expression == 'angry':
            pygame.draw.arc(self.screen, (150, 75, 0), (x - 300, y - 480 + adjusted_eyebrow_y, 600, 300), 3.14, 6.28, width=36)  # 生气的眉毛
        else:
            pygame.draw.arc(self.screen, (150, 75, 0), (x - 300, y - 480 + adjusted_eyebrow_y, 600, 300), 6.28, 3.14, width=36)  # 中立的眉毛

        # 绘制眼睫毛 - 修正角度范围为上半部分 (从 0 到 π)
        # if self.is_blinking == False:
        #     for i in range(11):  # 0到10的11个点
        #         angle = math.pi * i / 10  # 角度范围从0到π
        #         start_pos = (x + 200 * math.cos(angle), y - 160 * math.sin(angle))
        #         end_pos = (x + 230 * math.cos(angle), y - 180 * math.sin(angle))
        #         pygame.draw.line(self.screen, (0, 0, 0), start_pos, end_pos, 2)
            
    # def draw_mouth(self, x, y):
    #     # 绘制嘴巴
    #     adjusted_mouth_y = self.current_mouth_position
    #     if self.expression == 'surprised':
    #         pygame.draw.ellipse(self.screen, (0, 0, 0), (x - 120, y + 80 + adjusted_mouth_y, 240, 120))  # 张大嘴巴
    #     elif self.expression == 'angry':
    #         pygame.draw.arc(self.screen, (0, 0, 0), (x - 120, y + adjusted_mouth_y, 240, 120), 6.28, 3.14, width=12)  # 嘴巴下翘
    #     else:
    #         pygame.draw.arc(self.screen, (0, 0, 0), (x - 120, y + adjusted_mouth_y, 240, 120), 3.14, 6.28, width=12)  # 默认嘴巴

    def get_pupil_offset(self, direction):
        # 根据方向返回瞳孔的偏移量
        pupil_offset = {
            'left': (-120, 0),   # 向左看，增大偏移量
            'right': (120, 0),   # 向右看，增大偏移量
            'up': (0, -60),      # 向上看，增大偏移量
            'down': (0, 60),     # 向下看，增大偏移量
            'neutral': (0, 0)    # 中立（默认）
        }
        return pupil_offset.get(direction, (0, 0))

    def get_mouth_offset(self, expression):
        # 根据表情返回嘴巴的偏移量
        mouth_offset = {
            'surprised': -40,  # 张大嘴巴
            'angry': 20,       # 嘴巴下翘
            'neutral': 0       # 中立
        }
        return mouth_offset.get(expression, 0)

    def get_eyebrow_offset(self, expression):
        # 根据表情返回眉毛的偏移量
        eyebrow_offset = {
            'angry': -40,      # 生气的眉毛
            'neutral': 0       # 中立
        }
        return eyebrow_offset.get(expression, 0)

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

    def update_mouth_position(self):
        # 更新嘴巴位置，使其平滑过渡到目标位置
        dy = self.target_mouth_position - self.current_mouth_position
        if abs(dy) > self.mouth_speed:
            step_y = dy / abs(dy) * self.mouth_speed
            self.current_mouth_position += step_y
        else:
            self.current_mouth_position = self.target_mouth_position

    def update_eyebrow_position(self):
        # 更新眉毛位置，使其平滑过渡到目标位置
        dy = self.target_eyebrow_position - self.current_eyebrow_position
        if abs(dy) > self.eyebrow_speed:
            step_y = dy / abs(dy) * self.eyebrow_speed
            self.current_eyebrow_position += step_y
        else:
            self.current_eyebrow_position = self.target_eyebrow_position

    def run_pygame(self):
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

            
            # self.screen.fill((255, 228, 196))  # 肤色
            # self.screen.fill((245, 222, 179))  # 深肤色
            # self.screen.fill((204, 255, 229))  # 薄荷绿 
            # self.screen.fill((255, 218, 233))  # 蜜桃粉
            # self.screen.fill((255, 240, 245))  # 浅粉色     
            # selscreen.filllf.((230, 208, 255))  # 淡紫色
            # self.screen.fill((255, 253, 208))  # 淡黄色
            self.screen.fill((255, 245, 238))  # 极浅肤色

            


            # eye_x_left, eye_y_left = 530, 600
            # eye_x_right, eye_y_right = 1530, 600
            # mouth_x, mouth_y = 1030, 1000  # 嘴巴的中心位置
            
            # 调整眼睛位置，使其在画布中央
            screen_width, screen_height = self.screen.get_size()
            eye_y_center = screen_height // 2  # 垂直居中
            eye_x_left = screen_width // 4     # 左眼水平位置
            eye_x_right = screen_width * 3 // 4  # 右眼水平位置
            eye_y_left = eye_y_center
            eye_y_right = eye_y_center
            mouth_x, mouth_y = screen_width // 2, eye_y_center + 400  # 嘴巴的中心位置

            self.update_pupil_position()  # 更新瞳孔位置
            self.update_mouth_position()  # 更新嘴巴位置
            self.update_eyebrow_position()  # 更新眉毛位置

            # 更新眨眼进度
            if self.is_blinking:
                if self.blink_state == 'open':
                    self.blink_progress -= self.blink_speed * (1 / self.clock.get_fps())
                    if self.blink_progress <= 0:
                        self.blink_progress = 0
                        self.blink_state = 'closed'
                elif self.blink_state == 'closed':
                    self.blink_progress += self.blink_speed * (1 / self.clock.get_fps())
                    if self.blink_progress >= 1:
                        self.blink_progress = 1
                        self.blink_state = 'open'
                        self.is_blinking = False  # 结束眨眼

            self.draw_eye(eye_x_left, eye_y_left)
            self.draw_eye(eye_x_right, eye_y_right)
            # self.draw_mouth(mouth_x, mouth_y)
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