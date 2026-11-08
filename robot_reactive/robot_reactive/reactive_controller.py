"""Part B and C of the task: the reactive layer between Nav2 and the robot.

This node does not plan or steer on its own. It sits on Nav2's velocity chain:

    velocity_smoother -> /cmd_vel_smoothed -> [this node] -> /cmd_vel_reactive
        -> collision_monitor -> /cmd_vel -> robot

Normally it forwards Nav2's command untouched. The state machine in
reactive_fsm.py decides when to replace it with a stop or a turn.

Path blocking is judged by Nav2 itself (/is_path_valid) and any new path comes
from Nav2's own planner (/compute_path_to_pose).
"""
from dataclasses import fields
import math

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from robot_reactive.reactive_fsm import FsmConfig, Inputs, ReactiveFsm, ReplanStatus

ACTIVE_STATUSES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                   GoalStatus.STATUS_CANCELING)


class ReactiveController(Node):

    def __init__(self):
        super().__init__('reactive_controller')
        for f in fields(FsmConfig):
            self.declare_parameter(f.name, f.default)
        self.fsm = ReactiveFsm(FsmConfig(**{f.name: self.get_parameter(f.name).value
                                            for f in fields(FsmConfig)}))

        self.enabled = self.declare_parameter('enabled', True).value
        self.planner_id = self.declare_parameter('planner_id', 'GridBased').value
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.sensor_timeout = self.declare_parameter('sensor_timeout', 0.5).value
        path_check_rate = self.declare_parameter('path_check_rate', 4.0).value

        self.front = self.left = self.right = float('inf')
        self.last_sensor_time = None
        self.cmd_v = 0.0
        self.override = False
        self.nav_goals_active = {}

        self.plan = None
        self.plan_seq = 0
        self.path_valid = None
        self.check_started = None
        self.replan_status = ReplanStatus.NONE

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_reactive', 10)
        self.state_pub = self.create_publisher(String, '/reactive/state', 10)
        self.event_pub = self.create_publisher(String, '/reactive/events', 10)
        self.alt_path_pub = self.create_publisher(Path, '/reactive/alternative_path', 1)

        self.create_subscription(TwistStamped, '/cmd_vel_smoothed', self.on_cmd, 10)
        self.create_subscription(Float32MultiArray, '/reactive/obstacle_info', self.on_info, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        for action in ('navigate_to_pose', 'navigate_through_poses'):
            self.create_subscription(
                GoalStatusArray, f'/{action}/_action/status',
                lambda msg, name=action: self.on_status(name, msg),
                qos_profile_action_status_default)

        self.valid_client = self.create_client(IsPathValid, '/is_path_valid')
        self.plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        self.create_timer(0.05, self.tick)
        self.create_timer(1.0 / path_check_rate, self.check_path)
        self.get_logger().info(f'Reactive controller ready (enabled={self.enabled})')

    # ---- inputs ------------------------------------------------------------

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def nav_active(self) -> bool:
        return any(self.nav_goals_active.values())

    def on_status(self, action: str, msg: GoalStatusArray):
        self.nav_goals_active[action] = any(s.status in ACTIVE_STATUSES for s in msg.status_list)

    def on_info(self, msg: Float32MultiArray):
        self.front, self.left, self.right = msg.data
        self.last_sensor_time = self.now()

    def on_plan(self, msg: Path):
        if not msg.poses:
            return
        self.plan = msg
        self.plan_seq += 1
        self.path_valid = None      # unknown until Nav2 has checked this new path

    def on_cmd(self, msg: TwistStamped):
        self.cmd_v = msg.twist.linear.x
        if not self.enabled or not self.override:
            self.cmd_pub.publish(msg)

    def dist_to_goal(self):
        if self.plan is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame,
                                                rclpy.time.Time())
        except TransformException:
            return None
        goal = self.plan.poses[-1].pose.position
        return math.hypot(goal.x - t.transform.translation.x, goal.y - t.transform.translation.y)

    # ---- ask Nav2: is the current path still usable? ------------------------

    def check_path(self):
        now = self.now()
        if self.check_started is not None and now - self.check_started > 2.0:
            self.check_started = None   # lost reply; don't wait forever
        if not self.enabled or not self.nav_active or self.plan is None \
                or self.check_started is not None or not self.valid_client.service_is_ready():
            return
        self.check_started = now
        seq = self.plan_seq
        future = self.valid_client.call_async(IsPathValid.Request(path=self.plan))
        future.add_done_callback(lambda f: self.on_path_checked(f, seq))

    def on_path_checked(self, future, seq):
        self.check_started = None
        result = future.result()
        if result is not None and seq == self.plan_seq:     # ignore answers for an old path
            self.path_valid = result.is_valid

    # ---- ask Nav2's planner for a new path ----------------------------------

    def request_replan(self):
        if self.plan is None or not self.plan_client.server_is_ready():
            self.replan_status = ReplanStatus.FAILED
            return
        goal = ComputePathToPose.Goal()
        goal.goal = self.plan.poses[-1]
        goal.goal.header.frame_id = self.plan.header.frame_id
        goal.planner_id = self.planner_id
        goal.use_start = False      # plan from wherever the robot is now
        self.replan_status = ReplanStatus.PENDING
        self.plan_client.send_goal_async(goal).add_done_callback(self.on_replan_accepted)

    def on_replan_accepted(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.replan_status = ReplanStatus.FAILED
            return
        handle.get_result_async().add_done_callback(self.on_replan_result)

    def on_replan_result(self, future):
        response = future.result()
        if response.status == GoalStatus.STATUS_SUCCEEDED and response.result.path.poses:
            self.replan_status = ReplanStatus.SUCCEEDED
            self.alt_path_pub.publish(response.result.path)
        else:
            self.replan_status = ReplanStatus.FAILED

    # ---- main loop, 20 Hz ---------------------------------------------------

    def tick(self):
        if not self.enabled:
            self.state_pub.publish(String(data='DISABLED'))
            return
        now = self.now()
        if now == 0.0:
            return      # simulation clock not running yet

        sensor_ok = self.last_sensor_time is not None \
            and now - self.last_sensor_time <= self.sensor_timeout
        if self.nav_active and not sensor_ok:
            # Without LiDAR data we can't know it's safe, so don't let the robot drive.
            self.get_logger().warn('No fresh obstacle data, holding robot',
                                   throttle_duration_sec=2.0)
            self.override = True
            self.publish_twist(0.0, 0.0)
            self.state_pub.publish(String(data='NO_SENSOR'))
            return

        out = self.fsm.step(Inputs(
            now=now, front=self.front, left=self.left, right=self.right, cmd_v=self.cmd_v,
            nav_active=self.nav_active, path_valid=self.path_valid,
            replan_status=self.replan_status, dist_to_goal=self.dist_to_goal()))

        if out.request_replan:
            self.request_replan()
        if out.event:
            self.get_logger().info(out.event)
            self.event_pub.publish(String(data=out.event))
        self.override = out.override
        if out.override:
            self.publish_twist(out.v, out.w)
        self.state_pub.publish(String(data=out.state.value))

    def publish_twist(self, v: float, w: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
