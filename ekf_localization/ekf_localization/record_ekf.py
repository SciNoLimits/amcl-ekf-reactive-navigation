import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def get_yaw(q):
    return math.atan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


class Recorder(Node):
    def __init__(self, path):
        super().__init__('ekf_recorder')
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.file = open(path, 'w', newline='', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'timestamp_sec', 'x_m', 'y_m', 'yaw_rad',
            'linear_velocity_x_mps', 'angular_velocity_z_radps',
            'variance_x', 'variance_y', 'variance_yaw',
            'variance_vx', 'variance_wz'
        ])
        self.count = 0
        self.sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.callback, 20
        )

    def callback(self, msg):
        p = msg.pose.pose
        t = msg.twist.twist
        pc = msg.pose.covariance
        tc = msg.twist.covariance
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.writer.writerow([
            stamp, p.position.x, p.position.y, get_yaw(p.orientation),
            t.linear.x, t.angular.z,
            pc[0], pc[7], pc[35], tc[0], tc[35]
        ])
        self.count += 1

    def close(self):
        self.file.flush()
        self.file.close()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=0.0)
    opts, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = Recorder(opts.output)
    start = time.monotonic()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if opts.duration > 0 and time.monotonic() - start >= opts.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        print(f'Recorded {node.count} messages to {opts.output}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
