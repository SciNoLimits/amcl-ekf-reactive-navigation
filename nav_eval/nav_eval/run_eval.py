#!/usr/bin/env python3
"""Automated navigation test runner.

For every (repeat, goal): reset robot to the identical start pose, send a Nav2
NavigateToPose goal, record estimated pose (map->base TF), Gazebo ground truth,
covariance and the first planned path, then record a stationary phase.
Outputs (in results/<mode>_<timestamp>/): trials.csv, samples.csv, plans.json, params.json
"""
import csv
import json
import math
import os
import subprocess
import threading
import time
from datetime import datetime

import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}
SAMPLE_COLS = ['trial_id', 'phase', 't', 'est_x', 'est_y', 'est_yaw',
               'gt_x', 'gt_y', 'gt_yaw', 'cov_xx', 'cov_yy', 'cov_yaw']
TRIAL_COLS = ['trial_id', 'mode', 'repeat', 'goal', 'goal_x', 'goal_y', 'goal_yaw',
              'nav2_status', 'nav2_success', 'gt_final_dist', 'est_final_dist',
              'success', 'duration_s', 'recoveries', 'start_err', 'missing_samples']


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Evaluator(Node):
    def __init__(self):
        super().__init__('nav_eval')
        defaults = {
            'mode': 'amcl', 'output_root': 'results', 'repeats': 3, 'goal_timeout': 120.0,
            'goal_tolerance_xy': 0.30, 'stationary_secs': 10.0, 'sample_hz': 10.0,
            'settle_secs': 3.0, 'map_frame': 'map', 'base_frame': 'base_footprint',
            'nav_action': '/navigate_to_pose', 'plan_topic': '/plan',
            'cov_topic': '/amcl_pose', 'cov_type': 'pose_cov', 'world_name': 'default',
            'gt_topic': '/world/default/dynamic_pose/info',
            'gt_child_frame': 'turtlebot3_waffle_pi',
            'map_origin_in_world': [0.0, 0.0, 0.0], 'reset_mode': 'gz',
            'model_name': 'turtlebot3_waffle_pi', 'spawn_z': 0.01,
            'publish_initial_pose': True, 'initial_pose_topic': '/initialpose',
            'clear_costmaps': True, 'goals_file': '', 'gt_source': 'gz_cli',
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._gt_world = None
        self._gt_names_logged = False
        self._cov = (float('nan'),) * 3
        self._first_plan = None
        self._accept_plan = False
        self._recoveries = 0
        self.phase = None
        self._buf = []
        self._missing = 0

        self._gz_proc = None
        if self.p['gt_source'] == 'gz_cli':
            threading.Thread(target=self._gz_cli_loop, daemon=True).start()
        else:
            self.create_subscription(TFMessage, self.p['gt_topic'], self._gt_cb, 10)
        cov_t = Odometry if self.p['cov_type'] == 'odom' else PoseWithCovarianceStamped
        self.create_subscription(cov_t, self.p['cov_topic'], self._cov_cb, 10)
        self.create_subscription(Path, self.p['plan_topic'], self._plan_cb, 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              self.p['initial_pose_topic'], 10)
        self.nav = ActionClient(self, NavigateToPose, self.p['nav_action'])
        self.clear_g = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_l = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.create_timer(1.0 / self.p['sample_hz'], self._sample)

    # ------------------------------------------------------------------ callbacks
    def _gt_cb(self, msg):
        if not self._gt_names_logged:
            self._gt_names_logged = True
            names = sorted({t.child_frame_id for t in msg.transforms})
            self.get_logger().info(f'Ground-truth frames seen: {names}')
        for t in msg.transforms:
            if t.child_frame_id == self.p['gt_child_frame']:
                tr, q = t.transform.translation, t.transform.rotation
                self._gt_world = (tr.x, tr.y, yaw_of(q))

    @staticmethod
    def parse_gz_line(st, line):
        """Incremental parser of `gz topic -e` text output (Pose_V). Returns (name, x, y, yaw) or None."""
        s = line.strip()
        if not st['in']:
            if s == 'pose {':
                st.update({'in': True, 'sec': None, 'name': None,
                           'px': 0.0, 'py': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 0.0})
            return None
        if s.startswith('name:'):
            st['name'] = s.split(':', 1)[1].strip().strip('"')
        elif s == 'position {':
            st['sec'] = 'p'
        elif s == 'orientation {':
            st['sec'] = 'o'
        elif s == '}':
            if st['sec']:
                st['sec'] = None
            else:
                st['in'] = False
                q = type('Q', (), {'x': st['ox'], 'y': st['oy'], 'z': st['oz'], 'w': st['ow']})
                return (st['name'], st['px'], st['py'], yaw_of(q))
        elif st['sec'] and ':' in s:
            k, v = s.split(':', 1)
            try:
                st[st['sec'] + k.strip()] = float(v)
            except ValueError:
                pass
        return None

    def _gz_cli_loop(self):
        try:
            self._gz_proc = subprocess.Popen(['gz', 'topic', '-e', '-t', self.p['gt_topic']],
                                             stdout=subprocess.PIPE, text=True, bufsize=1)
        except Exception as e:
            self.get_logger().error(f'Cannot start gz topic: {e}')
            return
        st, seen = {'in': False}, set()
        for line in self._gz_proc.stdout:
            r = self.parse_gz_line(st, line)
            if r is None:
                continue
            name, x, y, yaw = r
            if name not in seen:
                seen.add(name)
                self.get_logger().info(f'Ground-truth entity seen: "{name}"')
            if name == self.p['gt_child_frame']:
                self._gz_gt = (x, y, yaw)
                self._gt_world = self._gz_gt

    def _cov_cb(self, msg):
        c = msg.pose.covariance
        self._cov = (c[0], c[7], c[35])

    def _plan_cb(self, msg):
        if self._accept_plan and self._first_plan is None and msg.poses:
            self._first_plan = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]

    def _feedback_cb(self, fb):
        self._recoveries = fb.feedback.number_of_recoveries

    # ------------------------------------------------------------------ helpers
    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_sim(self, secs):
        t0 = self.sim_now()
        while self.sim_now() - t0 < secs and rclpy.ok():
            time.sleep(0.05)

    def gt_in_map(self):
        if self._gt_world is None:
            return None
        ox, oy, oyaw = self.p['map_origin_in_world']
        x, y, yaw = self._gt_world
        dx, dy = x - ox, y - oy
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (c * dx + s * dy, -s * dx + c * dy, wrap(yaw - oyaw))

    def est_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.p['map_frame'], self.p['base_frame'], rclpy.time.Time())
        except Exception:
            return None
        tr = tf.transform.translation
        return (tr.x, tr.y, yaw_of(tf.transform.rotation))

    def _sample(self):
        if self.phase is None:
            return
        est, gt = self.est_in_map(), self.gt_in_map()
        if est is None or gt is None:
            self._missing += 1
            return
        self._buf.append([self._trial_id, self.phase, round(self.sim_now(), 3), *est, *gt, *self._cov])

    def wait_future(self, fut, timeout):
        t0 = self.sim_now()
        while not fut.done() and self.sim_now() - t0 < timeout and rclpy.ok():
            time.sleep(0.02)
        return fut.done()

    # ------------------------------------------------------------------ robot reset
    def reset_robot(self, start):
        x, y, yaw = start['x'], start['y'], start['yaw']
        if self.p['reset_mode'] == 'gz':
            ox, oy, oyaw = self.p['map_origin_in_world']
            c, s = math.cos(oyaw), math.sin(oyaw)
            wx, wy, wyaw = ox + c * x - s * y, oy + s * x + c * y, yaw + oyaw
            req = (f'name: "{self.p["model_name"]}", '
                   f'position: {{x: {wx}, y: {wy}, z: {self.p["spawn_z"]}}}, '
                   f'orientation: {{x: 0, y: 0, z: {math.sin(wyaw / 2)}, w: {math.cos(wyaw / 2)}}}')
            cmd = ['gz', 'service', '-s', f'/world/{self.p["world_name"]}/set_pose',
                   '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                   '--timeout', '3000', '--req', req]
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            if r.returncode != 0 or 'true' not in r.stdout.lower():
                self.get_logger().error(f'gz set_pose failed: {r.stdout} {r.stderr}')
        else:
            self.get_logger().warn('reset_mode=none: assuming robot is already at the start pose')
        self.wait_sim(1.0)
        if self.p['publish_initial_pose']:
            m = PoseWithCovarianceStamped()
            m.header.frame_id = self.p['map_frame']
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.05
            m.pose.covariance[35] = 0.02
            for _ in range(3):
                m.header.stamp = self.get_clock().now().to_msg()
                self.init_pub.publish(m)
                self.wait_sim(0.5)
        self.wait_sim(self.p['settle_secs'])
        if self.p['clear_costmaps']:
            for cli in (self.clear_g, self.clear_l):
                if cli.wait_for_service(timeout_sec=2.0):
                    self.wait_future(cli.call_async(ClearEntireCostmap.Request()), 5.0)

    # ------------------------------------------------------------------ one trial
    def run_trial(self, trial_id, repeat, goal, start):
        self._trial_id = trial_id
        self._buf, self._missing, self._recoveries = [], 0, 0
        self.reset_robot(start)

        est, gt = self.est_in_map(), self.gt_in_map()
        start_err = math.hypot(est[0] - gt[0], est[1] - gt[1]) if est and gt else float('nan')

        goal_msg = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.p['map_frame']
        ps.pose.position.x, ps.pose.position.y = goal['x'], goal['y']
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(goal['yaw'] / 2), math.cos(goal['yaw'] / 2)
        goal_msg.pose = ps

        self._first_plan, self._accept_plan = None, True
        self.phase = 'moving'
        t_start = self.sim_now()
        status = 'REJECTED'
        fut = self.nav.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        if self.wait_future(fut, 10.0) and fut.result().accepted:
            gh = fut.result()
            res = gh.get_result_async()
            if self.wait_future(res, self.p['goal_timeout']):
                status = STATUS_NAMES.get(res.result().status, str(res.result().status))
            else:
                status = 'TIMEOUT'
                self.wait_future(gh.cancel_goal_async(), 5.0)
                self.wait_future(res, 10.0)
        duration = self.sim_now() - t_start
        self._accept_plan = False

        # final ground-truth check at the end of the moving phase
        self.wait_sim(1.0)
        est, gt = self.est_in_map(), self.gt_in_map()
        gt_d = math.hypot(gt[0] - goal['x'], gt[1] - goal['y']) if gt else float('nan')
        est_d = math.hypot(est[0] - goal['x'], est[1] - goal['y']) if est else float('nan')
        nav2_ok = status == 'SUCCEEDED'
        verified = nav2_ok and gt_d <= self.p['goal_tolerance_xy']

        self.phase = 'stationary'
        self.wait_sim(self.p['stationary_secs'])
        self.phase = None

        row = dict(trial_id=trial_id, mode=self.p['mode'], repeat=repeat, goal=goal['name'],
                   goal_x=goal['x'], goal_y=goal['y'], goal_yaw=goal['yaw'], nav2_status=status,
                   nav2_success=int(nav2_ok), gt_final_dist=round(gt_d, 4),
                   est_final_dist=round(est_d, 4), success=int(verified),
                   duration_s=round(duration, 2), recoveries=self._recoveries,
                   start_err=round(start_err, 4), missing_samples=self._missing)
        return row, list(self._buf), self._first_plan

    # ------------------------------------------------------------------ main loop
    def run(self):
        goals_file = self.p['goals_file'] or os.path.join(
            get_package_share_directory('nav_eval'), 'config', 'goals.yaml')
        with open(goals_file) as f:
            cfg = yaml.safe_load(f)
        start, goals = cfg['start'], cfg['goals']
        if cfg.get('frame', 'map') == 'world':
            # goals are given in Gazebo world coordinates -> convert to map frame
            ox, oy, oyaw = self.p['map_origin_in_world']
            c, s_ = math.cos(oyaw), math.sin(oyaw)

            def w2m(x, y, yaw):
                dx, dy = x - ox, y - oy
                return c * dx + s_ * dy, -s_ * dx + c * dy, wrap(yaw - oyaw)
            goals = [dict(g, **dict(zip(('x', 'y', 'yaw'), w2m(g['x'], g['y'], g.get('yaw', 0.0)))))
                     for g in goals]
            if start == 'spawn':
                start = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}   # map origin == spawn pose
            else:
                start = dict(zip(('x', 'y', 'yaw'), w2m(start['x'], start['y'], start['yaw'])))
        elif start == 'spawn':
            start = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        log = self.get_logger()
        log.info('Waiting for sim clock, Nav2 action server, TF and ground truth...')
        t0 = time.time()
        while self.sim_now() == 0.0 and time.time() - t0 < 20:
            time.sleep(0.2)
        if self.sim_now() == 0.0:
            log.error('No /clock received. Is Gazebo running and use_sim_time true?')
            return
        if not self.nav.wait_for_server(timeout_sec=30.0):
            log.error(f'Action server {self.p["nav_action"]} not available. Is Nav2 up?')
            return
        t0 = time.time()
        while time.time() - t0 < 30 and (self.est_in_map() is None or self.gt_in_map() is None):
            time.sleep(0.5)
        if self.est_in_map() is None:
            log.error(f'TF {self.p["map_frame"]}->{self.p["base_frame"]} missing '
                      '(localization not running / initial pose not set?)')
            return
        if self.gt_in_map() is None:
            log.error(f'No ground truth for frame "{self.p["gt_child_frame"]}" on '
                      f'{self.p["gt_topic"]}. Check the bridge and the frame names logged above.')
            return

        out = os.path.join(self.p['output_root'],
                           f'{self.p["mode"]}_{datetime.now():%Y%m%d_%H%M%S}')
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, 'params.json'), 'w') as f:
            json.dump({'params': self.p, 'start': start, 'goals': goals}, f, indent=2)
        tf_ = open(os.path.join(out, 'trials.csv'), 'w', newline='')
        sf_ = open(os.path.join(out, 'samples.csv'), 'w', newline='')
        tw, sw = csv.DictWriter(tf_, TRIAL_COLS), csv.writer(sf_)
        tw.writeheader()
        sw.writerow(SAMPLE_COLS)
        plans = {}
        n_ok = n = 0
        total = self.p['repeats'] * len(goals)
        try:
            for rep in range(self.p['repeats']):
                for g in goals:
                    tid = f'r{rep}_{g["name"]}'
                    log.info(f'[{n + 1}/{total}] {tid} -> ({g["x"]}, {g["y"]})')
                    row, samples, plan = self.run_trial(tid, rep, g, start)
                    tw.writerow(row)
                    sw.writerows(samples)
                    tf_.flush()
                    sf_.flush()
                    plans[tid] = plan
                    n += 1
                    n_ok += row['success']
                    log.info(f'    {row["nav2_status"]}  gt_dist={row["gt_final_dist"]} m  '
                             f'verified_success={row["success"]}  ({n_ok}/{n})')
        finally:
            tf_.close()
            sf_.close()
            with open(os.path.join(out, 'plans.json'), 'w') as f:
                json.dump(plans, f)
            if n:
                log.info(f'DONE. Goal success rate: {n_ok}/{n} = {100.0 * n_ok / n:.1f}%  -> {out}')

    def shutdown(self):
        self.phase = None
        if self._gz_proc:
            self._gz_proc.terminate()


def main():
    rclpy.init()
    node = Evaluator()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted; partial results were saved.')
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
