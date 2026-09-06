import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from math import cos, sin


class EncoderProcessor(Node):

    def __init__(self):
        super().__init__('encoder_processor')

        # Subscribe to wheel joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publish odometry
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left = None
        self.prev_right = None

        self.get_logger().info(
            '✅ Encoder Processor started — listening on /joint_states'
        )

    def joint_callback(self, msg):

        try:
            # Find wheel joints
            left_idx = msg.name.index('front_left_joint')
            right_idx = msg.name.index('front_right_joint')

            left_pos = msg.position[left_idx]
            right_pos = msg.position[right_idx]

        except ValueError:
            return

        if self.prev_left is None:
            self.prev_left = left_pos
            self.prev_right = right_pos
            return

        # Wheel movement delta
        dl = left_pos - self.prev_left
        dr = right_pos - self.prev_right

        self.prev_left = left_pos
        self.prev_right = right_pos

        distance = (dl + dr) / 2.0

        self.x += distance * cos(self.theta)
        self.y += distance * sin(self.theta)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0
        )

        self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    node = EncoderProcessor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
