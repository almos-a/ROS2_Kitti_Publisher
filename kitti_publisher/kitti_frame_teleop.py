import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import threading
import sys
import termios
import tty

class KittiFrameTeleop(Node):
    def __init__(self):
        super().__init__('kitti_frame_teleop')
        self.pub_skip = self.create_publisher(Empty, '/kitti/skip_frame', 10)
        self.get_logger().info("Teleop ready: Press [SPACE] to skip frame, [Q] to quit.")

        # Start keyboard thread
        threading.Thread(target=self.keyboard_listener, daemon=True).start()

    def keyboard_listener(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key == ' ':
                    msg = Empty()
                    self.pub_skip.publish(msg)
                    self.get_logger().info('Skip frame requested.')
                elif key.lower() == 'q':
                    self.get_logger().info('Quit teleop.')
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = KittiFrameTeleop()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
