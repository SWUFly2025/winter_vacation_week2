import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy
import message_filters


class SyncImuLidarNode(Node):
    def __init__(self):
        super().__init__('sync_imu_lidar')
        self.get_logger().info("IMU & LiDAR Synchronization Node Started.")

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribers (실제 Gazebo 토픽/타입과 일치)
        self.imu_sub = message_filters.Subscriber(
            self,
            Imu,
            '/imu_plugin/out',
            qos_profile=sensor_qos
        )

        self.lidar_sub = message_filters.Subscriber(
            self,
            PointCloud2,
            '/lidar_plugin/out',
            qos_profile=sensor_qos
        )

        # Time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.imu_sub, self.lidar_sub],
            queue_size=30,
            slop=0.05
        )
        self.sync.registerCallback(self.sync_callback)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/synced_imu', sensor_qos)
        self.lidar_pub = self.create_publisher(PointCloud2, '/synced_points', sensor_qos)

    def sync_callback(self, imu_msg, lidar_msg):
        self.get_logger().info(
            f"Synced | IMU: {imu_msg.header.stamp.sec}.{imu_msg.header.stamp.nanosec} "
            f"LiDAR: {lidar_msg.header.stamp.sec}.{lidar_msg.header.stamp.nanosec}"
        )

        self.imu_pub.publish(imu_msg)
        self.lidar_pub.publish(lidar_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyncImuLidarNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
