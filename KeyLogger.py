import rclpy
from rclpy.node import Node
from std_msgs.msg import Char
from pynput.keyboard import Key, Listener


class KeyLogger(Node):

    def __init__(self):
        super().__init__('keylogger')
        self.publisher_press = self.create_publisher(Char, "/keylogger/press", 10)
        self.publisher_release = self.create_publisher(Char, "/keylogger/release", 10)

    def on_press(self, key):
        ch = Char()
        ch.data = ord(key.char)
        self.publisher_press.publish(ch)


    def on_release(self, key):
        ch = Char()
        ch.data = ord(key.char)
        self.publisher_release.publish(ch)

        if key == Key.esc:
            # Stop listener
            return False

    def run_getch(self):
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()

def main(args=None):
    rclpy.init(args=args)
    log = KeyLogger()
    rclpy.spin(log)
    log.run_getch()
    log.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()