import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener
from multiprocessing import Process, Manager


class MinimalPublisher(Node):

    def __init__(self,):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        manager = Manager()
        self.msg = manager.list((0.0, 0.0))
        self.i = 0

    def timer_callback(self):
        vel = Twist()
        vel.linear.x = self.msg[0]
        vel.linear.y = self.msg[1]
        self.publisher_.publish(vel)
        self.get_logger().info('Publishing: "%s"' % vel)
        self.i += 1

    def on_press(self, key):
        if key.char == 'w':
            self.msg[1] = 2.0
        elif key.char == 'a':
            self.msg[0] = -2.0
        elif key.char == 's':
            self.msg[1] = -2.0
        elif key.char == 'd':
            self.msg[0] = 2.0


    def on_release(self, key):
        if key.char == 'w':
            self.msg[1] = 0.0
        elif key.char == 'a':
            self.msg[0] = 0.0
        elif key.char == 's':
            self.msg[1] = 0.0
        elif key.char == 'd':
            self.msg[0] = 0.0
        elif key == Key.esc:
            # Stop listener
            return False

    def run_getch(self):
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()

def run_spin(pub):
    rclpy.spin("steering",executor=pub)

def run_in_parallel(args=None):
    rclpy.init(args=args)
    rclpy.create_node("steering")
    minimal_publisher = MinimalPublisher()
    p1 = Process(target=run_spin, args=(minimal_publisher,))
    p1.start()
    p2 = Process(target=minimal_publisher.run_getch)
    p2.start()
    p1.join()
    p2.join()
    minimal_publisher.destroy_node()
    rclpy.shutdown()

def main(args=None):
    run_in_parallel()


if __name__ == '__main__':
    main()
