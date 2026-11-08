"""Run one test scenario against a running simulation and record the results.

Usage (Nav2 and Gazebo must already be running, robot at its start pose):
    ros2 run robot_reactive scenario_runner S2 --label on --out results/

Writes <out>/<scenario>_<label>.json. All coordinates are in the map frame.
"""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path as FilePath
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from robot_reactive.obstacle_tool import move_box, remove_box, shuttle_positions, spawn_box

# Robot body rectangle in base_footprint (from the URDF).
BODY_X = (-0.197, 0.069)
BODY_Y = 0.153
GOAL_SE = (3.0, -1.5)
GOAL_FAR = (4.5, -0.4)
VELOCITY_CHAIN = ('cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel_reactive', 'cmd_vel')


@dataclass
class Scenario:
    description: str
    goal: Tuple[float, float]
    before_start: Optional[Callable] = None     # placed before the goal is sent
    on_trigger: Optional[Callable] = None       # called once the robot passes trigger_x
    trigger_x: float = 1.2
    timeout: float = 150.0
    ghost_region: Optional[Tuple[float, float, float, float]] = None  # xmin, xmax, ymin, ymax


def ahead_of(pose, forward, left):
    x, y, yaw = pose
    return (x + forward * math.cos(yaw) - left * math.sin(yaw),
            y + forward * math.sin(yaw) + left * math.cos(yaw))


def box_ahead(runner, pose):
    # The spawn call takes about 0.3 s and the robot keeps moving meanwhile, so
    # this ends up with the near face inside the stop range but not touching.
    x, y = ahead_of(pose, 0.85, 0.0)
    runner.add_box('sudden_box', x, y, 0.4, 0.4)


def box_very_close(runner, pose):
    # Same, but lands only about 0.15 m from the bumper, inside every stop threshold.
    x, y = ahead_of(pose, 0.65, 0.0)
    runner.add_box('close_box', x, y, 0.4, 0.4)


def box_offset(runner, pose):
    # Close to one side but outside the straight-ahead corridor.
    x, y = ahead_of(pose, 0.55, 0.50)
    runner.add_box('side_box', x, y, 0.4, 0.4)


def barrier(runner, _pose):
    # Wall from the top of the room down to y=-2.2, leaving a gap at the bottom.
    runner.add_box('barrier', 2.5, -0.195, 0.15, 4.0)


def start_shuttle(runner, _pose):
    # Starts so that it reaches the robot's line of travel as the robot gets there.
    runner.add_shuttle('moving_box', 2.2, -1.7, 1.0, 0.4)


def single_crossing(runner, _pose):
    # Like a person walking across once: same start and speed as S5, no return trip.
    runner.add_shuttle('moving_box', 2.2, -1.7, 1.0, 0.4, once=True)


def block_door(runner):
    runner.add_box('door_block', 0.65, -0.41, 0.2, 1.7)


SCENARIOS: Dict[str, Scenario] = {
    'S1': Scenario('No obstacle (baseline)', GOAL_SE),
    'S2': Scenario('Box suddenly appears ahead', GOAL_SE, on_trigger=box_ahead),
    'S2C': Scenario('Box appears very close ahead', GOAL_SE, on_trigger=box_very_close),
    'S3': Scenario('Box appears close on one side', GOAL_SE, on_trigger=box_offset),
    'S4': Scenario('Long barrier forces a detour', GOAL_FAR, on_trigger=barrier),
    'S5': Scenario('Moving box crosses the path', GOAL_FAR, on_trigger=start_shuttle,
                   trigger_x=0.9, ghost_region=(1.8, 2.6, -3.0, 1.2)),
    'S5B': Scenario('Box crosses the path once', GOAL_FAR, on_trigger=single_crossing,
                    trigger_x=0.9),
    'S6': Scenario('Door fully blocked, no route exists', GOAL_SE, before_start=block_door,
                   timeout=180.0),
}


def rect_clearance(pose, box) -> float:
    """Distance from the robot's body outline to an axis-aligned box."""
    x, y, yaw = pose
    cx, cy, sx, sy = box
    (x0, x1), w = BODY_X, BODY_Y
    outline = []
    for i in range(11):
        t = i / 10.0
        px = x0 + (x1 - x0) * t
        py = -w + 2.0 * w * t
        outline += [(px, w), (px, -w), (x0, py), (x1, py)]
    best = float('inf')
    for bx, by in outline:
        wx = x + bx * math.cos(yaw) - by * math.sin(yaw)
        wy = y + bx * math.sin(yaw) + by * math.cos(yaw)
        dx = max(abs(wx - cx) - sx / 2.0, 0.0)
        dy = max(abs(wy - cy) - sy / 2.0, 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


class Runner(Node):

    def __init__(self):
        super().__init__('scenario_runner',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.lock = threading.Lock()
        self.boxes: Dict[str, Tuple[float, float, float, float]] = {}
        self.stop_threads = threading.Event()
        self.threads: List[threading.Thread] = []

        self.state = 'UNKNOWN'
        self.front = None
        self.events: List[dict] = []
        self.plans: List[dict] = []
        self.alt_paths: List[dict] = []
        self.costmap: Optional[OccupancyGrid] = None
        self.t0: Optional[float] = None
        # Latest (linear, angular) at each stage of the velocity chain.
        self.layers = {name: [0.0, 0.0] for name in VELOCITY_CHAIN}
        self.cm_state = ''

        self.create_subscription(String, '/reactive/state', self.on_state, 10)
        self.create_subscription(String, '/reactive/events', self.on_event, 10)
        self.create_subscription(Float32MultiArray, '/reactive/obstacle_info', self.on_info, 10)
        for name in VELOCITY_CHAIN:
            self.create_subscription(TwistStamped, f'/{name}',
                                     lambda msg, n=name: self.on_layer(n, msg), 10)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state',
                                 self.on_cm_state, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(Path, '/reactive/alternative_path', self.on_alt, 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self) -> float:
        return 0.0 if self.t0 is None else self.now() - self.t0

    # --- recording -----------------------------------------------------------

    def on_state(self, msg):
        self.state = msg.data

    def on_layer(self, name, msg):
        self.layers[name] = [round(msg.twist.linear.x, 3), round(msg.twist.angular.z, 3)]

    def on_cm_state(self, msg):
        self.cm_state = f'{msg.action_type}:{msg.polygon_name}'

    def on_info(self, msg):
        self.front = float(msg.data[0])

    def on_event(self, msg):
        self.events.append({'t': round(self.elapsed(), 2), 'event': msg.data})

    @staticmethod
    def thin(path: Path) -> List[List[float]]:
        step = max(1, len(path.poses) // 200)
        return [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
                for p in path.poses[::step]]

    def on_plan(self, msg):
        if self.t0 is not None and msg.poses:
            self.plans.append({'t': round(self.elapsed(), 2), 'points': self.thin(msg)})

    def on_alt(self, msg):
        if self.t0 is not None and msg.poses:
            self.alt_paths.append({'t': round(self.elapsed(), 2), 'points': self.thin(msg)})

    def on_costmap(self, msg):
        self.costmap = msg

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    # --- obstacles -----------------------------------------------------------

    def add_box(self, name, x, y, sx, sy):
        spawn_box(name, x, y, sx, sy)
        with self.lock:
            self.boxes[name] = (x, y, sx, sy)
        self.events.append({'t': round(self.elapsed(), 2),
                            'event': f'SCENARIO placed {name} at ({x:.2f}, {y:.2f})'})

    def add_shuttle(self, name, x, y1, y2, speed, once=False):
        self.add_box(name, x, y1, 0.4, 0.4)
        crossing_time = abs(y2 - y1) / speed

        def loop():
            start = time.monotonic()
            while not self.stop_threads.is_set():
                elapsed = time.monotonic() - start
                if once and elapsed > crossing_time:
                    break       # one crossing done; the box stays at the far end
                y = shuttle_positions(y1, y2, speed, elapsed)
                move_box(name, x, y)
                with self.lock:
                    self.boxes[name] = (x, y, 0.4, 0.4)
        thread = threading.Thread(target=loop, daemon=True)
        thread.start()
        self.threads.append(thread)

    def clear_boxes(self):
        self.stop_threads.set()
        for thread in self.threads:
            thread.join(timeout=2.0)
        for name in list(self.boxes):
            remove_box(name)
        with self.lock:
            self.boxes.clear()

    def count_ghosts(self, region) -> Optional[int]:
        """Lethal costmap cells left in a region after the obstacle has gone."""
        if self.costmap is None:
            return None
        info = self.costmap.info
        xmin, xmax, ymin, ymax = region
        count = 0
        for row in range(info.height):
            y = info.origin.position.y + (row + 0.5) * info.resolution
            if not ymin <= y <= ymax:
                continue
            for col in range(info.width):
                x = info.origin.position.x + (col + 0.5) * info.resolution
                if xmin <= x <= xmax and self.costmap.data[row * info.width + col] == 100:
                    count += 1
        return count

    # --- running -------------------------------------------------------------

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until_ready(self):
        if not self.client.wait_for_server(timeout_sec=120.0):
            raise RuntimeError('navigate_to_pose action server not available')
        deadline = time.monotonic() + 60.0
        while self.pose() is None or self.state == 'UNKNOWN':
            if time.monotonic() > deadline:
                raise RuntimeError('no map->base_footprint transform or /reactive/state')
            self.spin_for(0.2)

    def run(self, name: str, label: str) -> dict:
        scenario = SCENARIOS[name]
        self.wait_until_ready()
        self.spin_for(2.0)

        if scenario.before_start:
            scenario.before_start(self)
            self.spin_for(3.0)      # give the costmaps time to see it

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = scenario.goal
        goal.pose.pose.orientation.w = 1.0

        self.t0 = self.now()
        future = self.client.send_goal_async(goal)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('goal rejected')
        result_future = handle.get_result_async()

        trajectory, min_clearance, triggered, travelled, last = [], float('inf'), False, 0.0, None
        trigger_time = None
        next_sample = 0.0
        while not result_future.done() and self.elapsed() < scenario.timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            t = self.elapsed()
            if t < next_sample:
                continue
            next_sample = t + 0.1
            pose = self.pose()
            if pose is None:
                continue
            if not triggered and scenario.on_trigger and pose[0] > scenario.trigger_x:
                triggered = True
                trigger_time = t
                scenario.on_trigger(self, pose)
            with self.lock:
                boxes = list(self.boxes.values())
            clearance = min((rect_clearance(pose, b) for b in boxes), default=None)
            if clearance is not None:
                min_clearance = min(min_clearance, clearance)
            if last is not None:
                travelled += math.hypot(pose[0] - last[0], pose[1] - last[1])
            last = pose
            trajectory.append({'t': round(t, 2), 'x': round(pose[0], 3), 'y': round(pose[1], 3),
                               'yaw': round(pose[2], 3), 'state': self.state,
                               'cmd': {k: list(v) for k, v in self.layers.items()},
                               'cm': self.cm_state,
                               'clearance': None if clearance is None else round(clearance, 3)})

        timed_out = not result_future.done()
        status = GoalStatus.STATUS_UNKNOWN
        if timed_out:
            handle.cancel_goal_async()
            self.spin_for(1.0)
        else:
            status = result_future.result().status
        duration = self.elapsed()
        boxes_used = dict(self.boxes)

        self.clear_boxes()
        ghosts = None
        if scenario.ghost_region:
            self.spin_for(4.0)      # let the LiDAR clear what it can see
            ghosts = self.count_ghosts(scenario.ghost_region)

        def count(prefix):
            return sum(1 for e in self.events if prefix in e['event'])

        final = last or (0.0, 0.0, 0.0)
        return {
            'scenario': name, 'label': label, 'description': scenario.description,
            'goal': list(scenario.goal), 'succeeded': status == GoalStatus.STATUS_SUCCEEDED,
            'status': 'TIMEOUT' if timed_out else {
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(status, str(status)),
            'time_s': round(duration, 2), 'distance_m': round(travelled, 3),
            'time_after_trigger_s': None if trigger_time is None else round(duration - trigger_time, 2),
            'goal_error_m': round(math.hypot(final[0] - scenario.goal[0],
                                             final[1] - scenario.goal[1]), 3),
            'min_clearance_m': None if min_clearance == float('inf') else round(min_clearance, 3),
            'contact': min_clearance < 0.01,
            'stops': count('-> STOP'), 'turns': count('-> TURN_'), 'replans': count('-> REPLAN'),
            'resumes': count('RESUME'), 'hand_backs': count('HAND_BACK'),
            'ghost_cells': ghosts,
            'boxes': {k: list(v) for k, v in boxes_used.items()},
            'events': self.events, 'trajectory': trajectory, 'plans': self.plans,
            'alt_paths': self.alt_paths,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario', choices=sorted(SCENARIOS))
    parser.add_argument('--label', default='on', help='e.g. on / off, used in the file name')
    parser.add_argument('--out', default='results')
    args = parser.parse_args()

    rclpy.init()
    runner = Runner()
    try:
        result = runner.run(args.scenario, args.label)
    finally:
        runner.clear_boxes()
    out = FilePath(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / f'{args.scenario}_{args.label}.json').write_text(json.dumps(result))

    summary = {k: result[k] for k in ('scenario', 'label', 'status', 'time_s', 'time_after_trigger_s', 'distance_m',
                                      'goal_error_m', 'min_clearance_m', 'contact', 'stops',
                                      'turns', 'replans', 'resumes', 'hand_backs', 'ghost_cells')}
    print(json.dumps(summary))
    for event in result['events']:
        print(f"  {event['t']:7.2f}  {event['event']}")
    runner.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
