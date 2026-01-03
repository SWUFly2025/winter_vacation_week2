import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped
import math

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_localization')

        # Subscribers
        self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            10
        )

        self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        # Publisher
        self.pub_pose = self.create_publisher(
            PoseStamped,
            '/ekf/pose',
            10
        )

        # 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.yaw = 0.0

        self.dt = 0.01  # 100Hz 가정

    def imu_callback(self, msg):
        # 위치 예측
        ax = msg.linear_acceleration.x
        self.vx += ax * self.dt
        self.x += self.vx * self.dt

        # yaw 예측 (회전)
        wz = msg.angular_velocity.z
        self.yaw += wz * self.dt

        self.publish_pose()

    def gps_callback(self, msg):
        # GPS로 위치 보정
        self.x = msg.latitude
        self.y = msg.longitude

    def publish_pose(self):
        pose = PoseStamped()

        # 위치
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0

        # yaw → quaternion
        pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.pub_pose.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    rclpy.shutdown()
