"""The reactive decision logic, as a state machine with no ROS imports.

It maps directly onto the behaviour table in the task description:

    No obstacle            NAVIGATING   pass Nav2's command through
    Obstacle detected      STOPPED      publish zero velocity
    Obstacle on one side   TURNING      rotate toward the freer side
    Planned path blocked   REPLANNING   hold and ask Nav2's planner for a path
    New path available     NAVIGATING   resume

Every override state has a timeout. When it expires control goes back to Nav2,
so Nav2's progress checker and recovery behaviours can still do their job.
"""
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class State(str, Enum):
    NAVIGATING = 'NAVIGATING'
    STOPPED = 'STOPPED'
    TURNING = 'TURNING'
    REPLANNING = 'REPLANNING'


class ReplanStatus(Enum):
    NONE = 0
    PENDING = 1
    SUCCEEDED = 2
    FAILED = 3


@dataclass
class FsmConfig:
    stop_distance: float = 0.35     # stop if something is this close ahead (m)
    clear_distance: float = 0.50    # ...and only call it clear beyond this (m)
    side_distance: float = 0.25     # a side obstacle closer than this triggers a turn (m)
    side_margin: float = 0.15       # sides must differ by this to have a "safer" side (m)
    turn_speed: float = 0.6         # rad/s while TURNING
    min_forward_speed: float = 0.02  # below this Nav2 isn't really driving forward (m/s)
    stop_hold: float = 0.6          # minimum time stopped, lets the costmap catch up (s)
    stop_timeout: float = 4.0       # s
    turn_timeout: float = 2.5       # s
    replan_timeout: float = 3.0     # s
    replan_retry: float = 1.0       # s between attempts if the planner failed
    replan_settle: float = 1.1      # wait for Nav2's 1 Hz behaviour tree to adopt the path (s)
    replan_cooldown: float = 2.0    # s after replanning before another can start
    grace_period: float = 3.0       # s after an override during which Nav2 may steer away
    grace_stop_distance: float = 0.15  # stop distance during the grace period (m)
    goal_ignore_distance: float = 0.45  # no side turns this close to the goal (m)


@dataclass
class Inputs:
    now: float
    front: float
    left: float
    right: float
    cmd_v: float                    # forward speed Nav2 is currently asking for
    nav_active: bool                # is a navigation goal running?
    path_valid: Optional[bool]      # None until Nav2 has answered for the latest path
    replan_status: ReplanStatus = ReplanStatus.NONE
    dist_to_goal: Optional[float] = None


@dataclass
class Output:
    state: State
    override: bool                  # True: publish (v, w). False: pass Nav2's command.
    v: float = 0.0
    w: float = 0.0
    request_replan: bool = False
    event: Optional[str] = None


class ReactiveFsm:

    def __init__(self, cfg: FsmConfig):
        self.cfg = cfg
        self.state = State.NAVIGATING
        self._entered = 0.0
        self._grace_until = float('-inf')
        self._cooldown_until = float('-inf')
        self._turn_dir = 0.0
        self._last_request = float('-inf')
        self._succeeded_at: Optional[float] = None

    # ---- helpers ------------------------------------------------------------

    def _go(self, state: State, now: float):
        self.state = state
        self._entered = now

    def _resume(self, now: float, event: str) -> Output:
        """Hand control back to Nav2 with a grace period to steer away."""
        self._go(State.NAVIGATING, now)
        self._grace_until = now + self.cfg.grace_period
        return Output(State.NAVIGATING, False, event=event)

    def _start_turn(self, i: Inputs, why: str) -> Output:
        self._turn_dir = 1.0 if i.left > i.right else -1.0  # +z is a left turn
        self._go(State.TURNING, i.now)
        side = 'LEFT' if self._turn_dir > 0 else 'RIGHT'
        return Output(State.TURNING, True, 0.0, self._turn_dir * self.cfg.turn_speed,
                      event=f'{why} L={i.left:.2f} R={i.right:.2f} -> TURN_{side}')

    def _start_replan(self, i: Inputs, why: str) -> Output:
        self._go(State.REPLANNING, i.now)
        self._last_request = i.now
        self._succeeded_at = None
        return Output(State.REPLANNING, True, request_replan=True, event=f'{why} -> REPLAN')

    def _has_safer_side(self, i: Inputs) -> bool:
        return abs(i.left - i.right) >= self.cfg.side_margin

    # ---- main step ----------------------------------------------------------

    def step(self, i: Inputs) -> Output:
        if not i.nav_active:
            # No goal running: nothing to supervise, so forget any override.
            self._go(State.NAVIGATING, i.now)
            return Output(State.NAVIGATING, False)
        return {
            State.NAVIGATING: self._navigating,
            State.STOPPED: self._stopped,
            State.TURNING: self._turning,
            State.REPLANNING: self._replanning,
        }[self.state](i)

    def _navigating(self, i: Inputs) -> Output:
        cfg = self.cfg
        in_grace = i.now < self._grace_until
        moving_forward = i.cmd_v > cfg.min_forward_speed
        limit = cfg.grace_stop_distance if in_grace else cfg.stop_distance

        # An obstacle beyond the goal doesn't matter: we stop before reaching it.
        obstacle_before_goal = i.dist_to_goal is None or i.dist_to_goal > i.front + 0.1
        if moving_forward and i.front < limit and obstacle_before_goal:
            self._go(State.STOPPED, i.now)
            return Output(State.STOPPED, True, event=f'OBSTACLE_AHEAD front={i.front:.2f} -> STOP')

        if i.path_valid is False and i.now >= self._cooldown_until:
            return self._start_replan(i, 'PATH_BLOCKED')

        near_goal = i.dist_to_goal is not None and i.dist_to_goal < cfg.goal_ignore_distance
        if moving_forward and not in_grace and not near_goal \
                and min(i.left, i.right) < cfg.side_distance and self._has_safer_side(i):
            return self._start_turn(i, 'SIDE_OBSTACLE')

        return Output(State.NAVIGATING, False)

    def _stopped(self, i: Inputs) -> Output:
        cfg = self.cfg
        elapsed = i.now - self._entered
        if elapsed >= cfg.stop_hold:
            if i.front >= cfg.clear_distance:
                return self._resume(i.now, f'FRONT_CLEAR front={i.front:.2f} -> RESUME')
            if i.path_valid is False:
                return self._start_replan(i, 'PATH_BLOCKED')
            if self._has_safer_side(i):
                return self._start_turn(i, 'BLOCKED_AHEAD')
        if elapsed >= cfg.stop_timeout:
            return self._start_replan(i, 'STOP_TIMEOUT')
        return Output(State.STOPPED, True)

    def _turning(self, i: Inputs) -> Output:
        cfg = self.cfg
        if i.front >= cfg.clear_distance and min(i.left, i.right) >= cfg.side_distance + 0.05:
            return self._resume(i.now, f'TURN_DONE front={i.front:.2f} -> RESUME')
        if i.now - self._entered >= cfg.turn_timeout:
            return self._start_replan(i, 'TURN_TIMEOUT')
        return Output(State.TURNING, True, 0.0, self._turn_dir * cfg.turn_speed)

    def _replanning(self, i: Inputs) -> Output:
        cfg = self.cfg
        if i.replan_status == ReplanStatus.SUCCEEDED and self._succeeded_at is None:
            self._succeeded_at = i.now

        if self._succeeded_at is not None and i.path_valid is True \
                and i.now - self._succeeded_at >= cfg.replan_settle:
            self._cooldown_until = i.now + cfg.replan_cooldown
            return self._resume(i.now, 'NEW_PATH_AVAILABLE -> RESUME')

        if i.now - self._entered >= cfg.replan_timeout:
            self._cooldown_until = i.now + cfg.replan_cooldown
            return self._resume(i.now, 'REPLAN_TIMEOUT -> HAND_BACK_TO_NAV2')

        retry = i.replan_status == ReplanStatus.FAILED \
            and i.now - self._last_request >= cfg.replan_retry
        if retry:
            self._last_request = i.now
        return Output(State.REPLANNING, True, request_replan=retry)
