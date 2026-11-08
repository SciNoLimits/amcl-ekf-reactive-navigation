#!/usr/bin/env python3
"""Record EKF and Gazebo ground truth during a Nav2 NavigateToPose trial."""

import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ExperimentRecorder(Node):
    def __init__(self, args):
        super().__init__('ekf_experiment_recorder')

        self.args = args
        self.phase = 'startup'

        os.makedirs(args.output_dir, exist_ok=True)

        self.ekf_file = open(
            os.path.join(args.output_dir, 'ekf.csv'),
            'w',
            newline='',
            buffering=1
        )

        self.gt_file = open(
            os.path.join(args.output_dir, 'ground_truth.csv'),
            'w',
            newline='',
            buffering=1
        )

        self.ekf_writer = csv.writer(self.ekf_file)
        self.gt_writer = csv.writer(self.gt_file)

        self.ekf_writer.writerow([
            'timestamp_sec', 'phase', 'x_m', 'y_m', 'yaw_rad',
            'vx_mps', 'wz_radps', 'variance_x', 'variance_y',
            'variance_yaw', 'variance_vx', 'variance_wz'
        ])

        self.gt_writer.writerow([
            'timestamp_sec', 'phase', 'x_m', 'y_m', 'z_m',
            'qx', 'qy', 'qz', 'qw', 'yaw_rad'
        ])

        self.ekf_count = 0
        self.gt_count = 0

        self.last_ekf_stamp = None
        self.last_gt_stamp = None

        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.on_ekf,
            100
        )

        self.create_subscription(
            PoseStamped,
            '/ground_truth/pose',
            self.on_ground_truth,
            100
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

    def on_ekf(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        cov = msg.pose.covariance
        tcov = msg.twist.covariance

        stamp = stamp_seconds(msg.header.stamp)
        self.last_ekf_stamp = stamp

        self.ekf_writer.writerow([
            stamp,
            self.phase,
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
            twist.linear.x,
            twist.angular.z,
            cov[0],
            cov[7],
            cov[35],
            tcov[0],
            tcov[35]
        ])

        self.ekf_count += 1

    def on_ground_truth(self, msg):
        pose = msg.pose

        stamp = stamp_seconds(msg.header.stamp)
        self.last_gt_stamp = stamp

        self.gt_writer.writerow([
            stamp,
            self.phase,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            yaw_from_quaternion(pose.orientation)
        ])

        self.gt_count += 1

    def close(self):
        for file_obj in (self.ekf_file, self.gt_file):
            if not file_obj.closed:
                file_obj.flush()
                file_obj.close()


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_for_topics(node):
    deadline = time.monotonic() + node.args.stream_timeout

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

        if (
            node.last_ekf_stamp is not None
            and node.last_gt_stamp is not None
        ):
            node.get_logger().info(
                f"Received both streams: "
                f"EKF={node.last_ekf_stamp:.3f}s, "
                f"Ground truth={node.last_gt_stamp:.3f}s"
            )
            return

    raise RuntimeError(
        f"Timed out waiting for data streams. "
        f"EKF received: {node.last_ekf_stamp is not None}; "
        f"Ground truth received: {node.last_gt_stamp is not None}. "
        f"EKF samples: {node.ekf_count}; "
        f"Ground-truth samples: {node.gt_count}."
    )


def execute_nav_goal(node):
    node.get_logger().info(
        "Waiting for Nav2 action server..."
    )

    if not node.nav_client.wait_for_server(
        timeout_sec=node.args.server_timeout
    ):
        raise RuntimeError(
            "Nav2 action server /navigate_to_pose unavailable"
        )

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()

    goal.pose.header.frame_id = node.args.goal_frame
    goal.pose.header.stamp = node.get_clock().now().to_msg()

    goal.pose.pose.position.x = node.args.goal_x
    goal.pose.pose.position.y = node.args.goal_y

    half_yaw = 0.5 * node.args.goal_yaw

    goal.pose.pose.orientation.z = math.sin(half_yaw)
    goal.pose.pose.orientation.w = math.cos(half_yaw)

    node.phase = 'navigation'

    node.get_logger().info(
        f"Sending goal: "
        f"x={node.args.goal_x:.3f}, "
        f"y={node.args.goal_y:.3f}, "
        f"yaw={node.args.goal_yaw:.3f}, "
        f"frame={node.args.goal_frame}"
    )

    send_future = node.nav_client.send_goal_async(goal)

    while rclpy.ok() and not send_future.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    goal_handle = send_future.result()

    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError("Nav2 rejected the goal")

    node.get_logger().info("Nav2 accepted the goal.")

    result_future = goal_handle.get_result_async()

    deadline = time.monotonic() + node.args.navigation_timeout

    while rclpy.ok() and not result_future.done():

        if time.monotonic() > deadline:
            node.get_logger().warning(
                "Navigation timeout reached. Requesting cancellation."
            )

            cancel_future = goal_handle.cancel_goal_async()

            while rclpy.ok() and not cancel_future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            raise TimeoutError(
                "Nav2 goal timed out and cancellation was requested"
            )

        rclpy.spin_once(node, timeout_sec=0.1)

    result = result_future.result()

    if result is None:
        raise RuntimeError("No result received from Nav2")

    return result.status


def main(argv=None):
    parser = argparse.ArgumentParser()

    parser.add_argument('--goal-x', type=float, required=True)
    parser.add_argument('--goal-y', type=float, required=True)
    parser.add_argument('--goal-yaw', type=float, default=0.0)

    parser.add_argument('--goal-frame', default='map')
    parser.add_argument('--output-dir', default='ekf_experiment')

    parser.add_argument(
        '--baseline-seconds',
        type=float,
        default=30.0
    )

    parser.add_argument(
        '--final-stationary-seconds',
        type=float,
        default=15.0
    )

    parser.add_argument(
        '--stream-timeout',
        type=float,
        default=30.0
    )

    parser.add_argument(
        '--server-timeout',
        type=float,
        default=60.0
    )

    parser.add_argument(
        '--navigation-timeout',
        type=float,
        default=240.0
    )

    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)

    node = None

    try:
        node = ExperimentRecorder(args)

        node.get_logger().info(
            "Waiting for EKF and Gazebo ground-truth streams"
        )

        wait_for_topics(node)

        # Phase 1: Stationary baseline
        node.phase = 'baseline'

        node.get_logger().info(
            f"Recording stationary baseline for "
            f"{args.baseline_seconds:.1f} seconds"
        )

        spin_for(node, args.baseline_seconds)

        # Phase 2: Autonomous navigation
        status = execute_nav_goal(node)

        node.get_logger().info(
            f"Nav2 action terminal status: {status}"
        )

        # Phase 3: Stationary after navigation
        node.phase = 'final_stationary'

        node.get_logger().info(
            f"Recording final stationary phase for "
            f"{args.final_stationary_seconds:.1f} seconds"
        )

        spin_for(node, args.final_stationary_seconds)

        node.get_logger().info(
            f"Saved {node.ekf_count} EKF samples and "
            f"{node.gt_count} ground-truth samples in "
            f"{os.path.abspath(args.output_dir)}"
        )

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                "Experiment interrupted by user."
            )

    except Exception as exc:
        if node is not None:
            node.get_logger().error(
                f"Experiment failed: {exc}"
            )
        raise

    finally:
        if node is not None:
            node.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
