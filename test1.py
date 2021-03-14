import rclpy
import sys
import tty
import termios
from rclpy.node import Node
from geometry_msgs.msg import Twist


from std_msgs.msg import String

orig_settings = termios.tcgetattr(sys.stdin)


class MinimalPublisher(Node):

    def __init__(self, msg):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.msg = msg

    def timer_callback(self):
        c = sys.stdin.read(1)[0]
        if (c == "w"):
            self.msg.data = "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        self.get_logger().info('Publishing: "%s"' % self.msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    msg = Twist
    minimal_publisher = MinimalPublisher(msg)

    rclpy.spin(minimal_publisher)

    tty.setcbreak(sys.stdin)
    while 1:
        minimal_publisher.publish(minimal_publisher.msg)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()