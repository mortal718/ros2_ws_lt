import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import sys
import math
import os
from Xlib import display, X

class EyeDrawer(Node):
    def __init__(self):
        super().__init__('eye_drawer')
        self.get_logger().info('Eye Drawer Node Initialized')
        self.init_pygame()
        self.direction = 'center'
        
        # 左右眼瞳孔位置控制
        self.left_pupil_pos = -280
        self.right_pupil_pos = 120
        self.left_pupil_pos_y = 120
        self.right_pupil_pos_y = 120
        self.target_left_pos = -280
        self.target_right_pos = 120
        self.target_left_pos_y = 120
        self.target_right_pos_y = 120
        self.start_left_pos = -280
        self.start_right_pos = 120
        self.start_left_pos_y = 120
        self.start_right_pos_y = 120
        self.default_move_duration = 0.3  # 默认移动时间
        self.move_duration = self.default_move_duration
        self.move_timer = 0
        self.is_moving = False
        
        # 眨眼相关变量
        self.default_blink_duration = 0.2  # 默认眨眼时间
        self.blink_duration = self.default_blink_duration
        self.blink_state = 'open'
        self.blink_progress = 0
        self.blink_timer = 0
        self.is_blinking = False
        
        # 拖动相关变量
        self.dragging = False
        self.drag_start_pos = None
        self.window_pos = [0, 0]

        self.subscription = self.create_subscription(
            String,
            'eye_expression',
            self.expression_callback,
            10
        )

    def expression_callback(self, msg):
        command = msg.data
        parts = command.split()
        
        if parts[0] == 'blink':
            # 重置为默认眨眼时间
            self.blink_duration = self.default_blink_duration
            if len(parts) > 1:
                try:
                    self.blink_duration = float(parts[1])
                except ValueError:
                    pass
            self.is_blinking = True
            self.blink_state = 'open'
            self.blink_progress = 0
            self.blink_timer = 0
        else:
            direction = parts[0]
            # 重置为默认移动时间
            self.move_duration = self.default_move_duration
            if len(parts) > 1:
                try:
                    self.move_duration = float(parts[1])
                except ValueError:
                    pass
            
            self.start_left_pos = self.left_pupil_pos
            self.start_right_pos = self.right_pupil_pos
            self.start_left_pos_y = self.left_pupil_pos_y
            self.start_right_pos_y = self.right_pupil_pos_y
            
            if direction == 'left':
                self.target_left_pos = -610
                self.target_right_pos = 100
                self.target_left_pos_y = 220   # 左眼向下移动
                self.target_right_pos_y = 180  # 右眼不动
            elif direction == 'right':
                self.target_left_pos = -260
                self.target_right_pos = 450
                self.target_left_pos_y = 180   # 左眼不动
                self.target_right_pos_y = 220  # 右眼向下移动
            elif direction == 'center':
                self.target_left_pos = -280
                self.target_right_pos = 120
                self.target_left_pos_y = 120   # 恢复原始高度
                self.target_right_pos_y = 120
                
            self.is_moving = True
            self.move_timer = 0

    def init_pygame(self):
        pygame.init()
        d = display.Display()
        screen = d.screen()
        screen_width = screen.width_in_pixels
        screen_height = screen.height_in_pixels
        
        window_x = (screen_width - 1930) // 2
        window_y = (screen_height - 1080) // 2
        os.environ['SDL_VIDEO_WINDOW_POS'] = f"{window_x},{window_y}"
        
        self.screen = pygame.display.set_mode((1930, 1080), pygame.NOFRAME)
        pygame.display.set_caption('')
        self.clock = pygame.time.Clock()

    def draw_eye(self, x, y, angle=0):
        eye_surface = pygame.Surface((1000, 1000), pygame.SRCALPHA)
        eye_width = 700
        eye_height = 700
        eye_x = (1000 - eye_width) // 2
        eye_y = (1000 - eye_height) // 2
        center_y = eye_y + eye_height // 2
        
        if self.is_blinking:
            current_height = eye_height * (1 - self.blink_progress)
            current_y = center_y - current_height // 2
            
            pygame.draw.ellipse(eye_surface, (50, 50, 50), 
                              (eye_x, current_y, eye_width, current_height))
            pygame.draw.arc(eye_surface, (255, 255, 0), 
                          (eye_x, current_y, eye_width, current_height), 
                          0, 2 * math.pi, width=38)
        else:
            pygame.draw.ellipse(eye_surface, (50, 50, 50), 
                              (eye_x, eye_y, eye_width, eye_height))
            pygame.draw.arc(eye_surface, (255, 255, 0), 
                          (eye_x, eye_y, eye_width, eye_height), 
                          0, 2 * math.pi, width=38)
            
            if x < 1000:  # 左眼
                pygame.draw.ellipse(eye_surface, (255, 255, 255), 
                                (eye_x + eye_width + self.left_pupil_pos, 
                                eye_y + self.left_pupil_pos_y, 160, 160))
            else:  # 右眼
                pygame.draw.ellipse(eye_surface, (255, 255, 255), 
                                (eye_x + self.right_pupil_pos, 
                                eye_y + self.right_pupil_pos_y, 160, 160))
                
        rotated_surface = pygame.transform.rotate(eye_surface, math.degrees(angle))
        rot_rect = rotated_surface.get_rect(center=(x, y))
        self.screen.blit(rotated_surface, rot_rect)

    def run_pygame(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    self.get_logger().info('Pygame window closed.')
                    self.destroy_node()
                    rclpy.shutdown()
                    pygame.quit()
                    sys.exit()
                
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self.dragging = True
                        self.drag_start_pos = event.pos
                        d = display.Display()
                        window = d.get_input_focus().focus
                        geometry = window.get_geometry()
                        self.window_pos = [geometry.x, geometry.y]
                
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        self.dragging = False
                
                elif event.type == pygame.MOUSEMOTION:
                    if self.dragging:
                        current_pos = event.pos
                        if self.drag_start_pos:
                            dx = current_pos[0] - self.drag_start_pos[0]
                            dy = current_pos[1] - self.drag_start_pos[1]
                            self.window_pos[0] += dx
                            self.window_pos[1] += dy
                            d = display.Display()
                            window = d.get_input_focus().focus
                            window.configure(x=self.window_pos[0], y=self.window_pos[1])
                            d.sync()
                        self.drag_start_pos = current_pos

            self.screen.fill((0, 0, 0))

            center_x = 1930 // 2
            center_y = 1080 // 2
            eye_spacing = 1000
            
            eye_x_left = center_x - eye_spacing // 2
            eye_x_right = center_x + eye_spacing // 2
            eye_y_left = center_y
            eye_y_right = center_y

            if self.is_moving:
                dt = 1 / self.clock.get_fps()
                self.move_timer += dt
                progress = min(self.move_timer / self.move_duration, 1.0)
                
                self.left_pupil_pos = self.start_left_pos + (self.target_left_pos - self.start_left_pos) * progress
                self.right_pupil_pos = self.start_right_pos + (self.target_right_pos - self.start_right_pos) * progress
                self.left_pupil_pos_y = self.start_left_pos_y + (self.target_left_pos_y - self.start_left_pos_y) * progress
                self.right_pupil_pos_y = self.start_right_pos_y + (self.target_right_pos_y - self.start_right_pos_y) * progress
                
                if progress >= 1.0:
                    self.is_moving = False
                    self.left_pupil_pos = self.target_left_pos
                    self.right_pupil_pos = self.target_right_pos
                    self.left_pupil_pos_y = self.target_left_pos_y
                    self.right_pupil_pos_y = self.target_right_pos_y
            
            self.draw_eye(eye_x_left, eye_y_left, 0)
            self.draw_eye(eye_x_right, eye_y_right, 0)

            if self.is_blinking:
                dt = 1 / self.clock.get_fps()
                self.blink_timer += dt
                
                if self.blink_state == 'open':
                    self.blink_progress = self.blink_timer / (self.blink_duration * 0.5)
                    if self.blink_progress >= 1:
                        self.blink_progress = 1
                        self.blink_state = 'closed'
                        self.blink_timer = 0
                elif self.blink_state == 'closed':
                    self.blink_progress = 1 - (self.blink_timer / (self.blink_duration * 0.5))
                    if self.blink_progress <= 0:
                        self.blink_progress = 0
                        self.blink_state = 'open'
                        self.is_blinking = False
                        self.blink_timer = 0

            pygame.display.flip()
            self.clock.tick(30)
            rclpy.spin_once(self, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    node = EyeDrawer()
    node.run_pygame()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()