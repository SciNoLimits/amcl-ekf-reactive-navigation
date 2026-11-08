import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovarianceAdapter(Node):
    def __init__(self):
        super().__init__('imu_covariance_adapter')

        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('angular_velocity_variance', 1.0e-4)

        self.frame = self.get_parameter('imu_frame').value
        self.variance = float(
            self.get_parameter('angular_velocity_variance').value
        )

        self.pub = self.create_publisher(Imu, '/imu/ekf', 10)
        self.sub = self.create_subscription(
            Imu, '/imu', self.callback, 10
        )

        self.get_logger().info('IMU covariance adapter started')

    def callback(self, msg):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.frame

        if all(v == 0.0 for v in out.angular_velocity_covariance):
            cov = [0.0] * 9
            cov[0] = self.variance
            cov[4] = self.variance
            cov[8] = self.variance
            out.angular_velocity_covariance = cov

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
