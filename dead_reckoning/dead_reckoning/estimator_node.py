# standard imports including math for sin and cos functions
import rclpy
from rclpy.node import Node
import math

# specific imports for topics and messages
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy


class StateEstimator(Node):
    def __init__(self):
        super().__init__('state_estimator')
        # subscribe to commanded velocity and IMU topics /cmd_vel and /imu
        self.subscription_cmd_vel = self.create_subscription(TwistStamped,'cmd_vel',
                                                    self.estimator_callback_a, 10)
        self.subscription_imu = self.create_subscription(Imu,'imu',
                                                        self.estimator_callback_b, 10)
        
        # publishers 

        # path qos profile for adding Transient Local
        path_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

         # publishers
        self.dr_odom_pub = self.create_publisher(Odometry, '/dead_reckoning/odom', 10)
        self.dr_path_pub = self.create_publisher(Path, '/dead_reckoning/path', path_qos)
        self.imu_odom_pub = self.create_publisher(Odometry, '/imu_integration/odom', 10)
        self.imu_path_pub = self.create_publisher(Path, '/imu_integration/path', path_qos)

        # state variables dead reckoning
        self.dr_x = 0.0
        self.dr_y = 0.0
        self.dr_theta = 0.0
        self.last_time_a = None

        # state variables IMU integration
        self.imu_v = 0.0
        self.imu_vx = 0.0
        self.imu_vy = 0.0
        self.imu_theta = 0.0
        self.imu_x = 0.0
        self.imu_y = 0.0
        self.last_time_b = None

        # path object
        self.dr_path = Path()
        self.imu_path = Path()
        self.dr_path.header.frame_id = "odom"
        self.imu_path.header.frame_id = "odom"

        # state equations (dead reckoning)
        # θ(t+Δt) = θ(t) + ω·Δt
        # x(t+Δt) = x(t) + v·cos(θ)·Δt
        # y(t+Δt) = y(t) + v·sin(θ)·Δt

        # state equations (IMU)
        # θ(t+Δt) = θ(t) + ωz·Δt
        # v(t+Δt) = v(t) + a·Δt
        # p(t+Δt) = p(t) + v·Δt

    def estimator_callback_a(self, msg):
        # handle determining of deltat.
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time_a is None:
            self.last_time_a = time
            return
        dt = time - self.last_time_a
        self.last_time_a = time

        # grab current state from topic
        v = msg.twist.linear.x
        omega = msg.twist.angular.z

        # compute current state with provided equations
        self.dr_theta += omega * dt
        self.dr_x += v * math.cos(self.dr_theta) * dt
        self.dr_y += v * math.sin(self.dr_theta) * dt


        # setup and publish odometry
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.dr_x
        odom.pose.pose.position.y = self.dr_y 
        odom.pose.pose.orientation.z = math.sin(self.dr_theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.dr_theta / 2.0)
        self.dr_odom_pub.publish(odom)

        # setup and publish path
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = self.dr_x
        pose.pose.position.y = self.dr_y
        pose.pose.orientation.z = math.sin(self.dr_theta / 2.0)
        pose.pose.orientation.w = math.cos(self.dr_theta / 2.0)
        self.dr_path.poses.append(pose)
        self.dr_path.header.stamp = msg.header.stamp
        self.dr_path_pub.publish(self.dr_path)
        

    def estimator_callback_b(self, msg):
        # handle determining of deltat.
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time_b is None:
            self.last_time_b = time
            return
        dt = time - self.last_time_b
        self.last_time_b = time

        # grab current state from topic
        self.imu_theta += msg.angular_velocity.z * dt
        ax_world = msg.linear_acceleration.x * math.cos(self.imu_theta) - msg.linear_acceleration.y * math.sin(self.imu_theta)
        ay_world = msg.linear_acceleration.x * math.sin(self.imu_theta) + msg.linear_acceleration.y * math.cos(self.imu_theta)
        
        # compute current state with provided equations
        self.imu_vx += ax_world * dt
        self.imu_vy += ay_world * dt
        self.imu_x += self.imu_vx * dt
        self.imu_y += self.imu_vy * dt

        # setup and publish odometry
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.imu_x
        odom.pose.pose.position.y = self.imu_y
        odom.pose.pose.orientation.z = math.sin(self.imu_theta / 2.0) 
        odom.pose.pose.orientation.w = math.cos(self.imu_theta / 2.0) 
        self.imu_odom_pub.publish(odom)   

        # setup and publish path
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = self.imu_x
        pose.pose.position.y = self.imu_y
        pose.pose.orientation.z = math.sin(self.imu_theta / 2.0)
        pose.pose.orientation.w = math.cos(self.imu_theta / 2.0)
        self.imu_path.poses.append(pose)
        self.imu_path.header.stamp = msg.header.stamp
        self.imu_path_pub.publish(self.imu_path)      
        

# main function to run the node
def main(args=None):
    rclpy.init(args=args)
    state_estimator = StateEstimator()
    rclpy.spin(state_estimator)
    state_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()