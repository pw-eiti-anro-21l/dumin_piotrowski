#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('preedicted_tip_position_NONKDL')
        self.publisher_ = self.create_publisher(PoseStamped, 'manipulator_tip_position_NONKDL', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback_joint_states, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.manipulator_position = PoseStamped()

        self.i = 0

    def calculate_manipulator_end_position(self, list_of_positions):

        base_position = [0.5, 0.5, 1.5]
        end_position = []


        end_position.append(base_position[0] + list_of_positions[0][1])
        end_position.append(base_position[1] - list_of_positions[1][1])
        end_position.append(base_position[2] - list_of_positions[2][1])

        return end_position

    def timer_callback(self):

        self.publisher_.publish(self.manipulator_position)
        # self.get_logger().info('Publishing: "%s"' % vel)
        self.i += 1

    def listener_callback_joint_states(self, msg):

        position_list = []

        for i in range(len(msg.name)):
            position_list.append([msg.name[i], msg.position[i]])

        end_position = self.calculate_manipulator_end_position(position_list)

        now = self.get_clock().now()
        self.manipulator_position.header.stamp = now.to_msg()
        self.manipulator_position.header.frame_id = 'odom'

        self.manipulator_position.pose.position.x = end_position[0]
        self.manipulator_position.pose.position.y = end_position[1]
        self.manipulator_position.pose.position.z = end_position[2]

        self.manipulator_position.pose.orientation.x = -0.7071054825064662
        self.manipulator_position.pose.orientation.y = 0.7071080798594737
        self.manipulator_position.pose.orientation.z = 0.0
        self.manipulator_position.pose.orientation.w = 0.000002597

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
