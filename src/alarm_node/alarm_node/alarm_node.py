import rclpy
from rclpy.node import Node
import pygame
import os

class AlarmNode(Node):
    def __init__(self):
        super().__init__('alarm_node')
        self.get_logger().info('Alarm node has started.')

        # Initialize pygame mixer
        pygame.mixer.init()

        # Path to your MP3 file
        mp3_path = os.path.join(os.path.dirname(__file__), 'alarm.mp3')

        # Load and play the audio
        try:
            pygame.mixer.music.load(mp3_path)
            pygame.mixer.music.play()

            self.get_logger().info('Playing alarm sound...')

            # Wait while sound is playing
            self.timer = self.create_timer(0.1, self.check_if_done)
        except Exception as e:
            self.get_logger().error(f'Failed to play sound: {e}')

    def check_if_done(self):
        if not pygame.mixer.music.get_busy():
            self.get_logger().info('Alarm finished.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AlarmNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

