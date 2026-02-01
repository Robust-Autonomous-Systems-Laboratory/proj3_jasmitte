import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class StateEstimator(Node):
    def __init__(self):
        super().__init__('state_estimator')
        # subscribe to commanded velocity and IMU topics /cmd_vel and /imu
        self.subscription_cmd_vel = self.create_subscription(String,'cmd_vel',
                                                    self.estimator_callback_a, 10)
        self.subscription_imu = self.create_subscription(String,'imu',
                                                        self.estimator_callback_b, 10)
        self.subscription_cmd_vel  # prevent unused variable warning
        self.subscription_imu  # prevent unused variable warning

    def estimator_callback_a(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def estimator_callback_b(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    state_estimator = StateEstimator()
    rclpy.spin(state_estimator)
    state_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()