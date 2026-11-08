"""Unit tests for the state machine: one group per row of the task's table."""
from robot_reactive.reactive_fsm import FsmConfig, Inputs, ReactiveFsm, ReplanStatus, State

CFG = FsmConfig()
FAR = 2.0


def inputs(now, front=FAR, left=FAR, right=FAR, cmd_v=0.3, nav_active=True, path_valid=True,
           replan_status=ReplanStatus.NONE, dist_to_goal=5.0):
    return Inputs(now, front, left, right, cmd_v, nav_active, path_valid, replan_status,
                  dist_to_goal)


def stopped_fsm():
    fsm = ReactiveFsm(CFG)
    assert fsm.step(inputs(0.0, front=0.3)).state == State.STOPPED
    return fsm


def replanning_fsm():
    fsm = ReactiveFsm(CFG)
    out = fsm.step(inputs(0.0, path_valid=False))
    assert out.state == State.REPLANNING and out.request_replan
    return fsm


# --- No obstacle -> continue navigation

def test_no_obstacle_passes_nav2_command():
    out = ReactiveFsm(CFG).step(inputs(0.0))
    assert out.state == State.NAVIGATING and not out.override


def test_unknown_path_validity_is_not_treated_as_blocked():
    assert ReactiveFsm(CFG).step(inputs(0.0, path_valid=None)).state == State.NAVIGATING


def test_no_goal_means_no_override():
    out = ReactiveFsm(CFG).step(inputs(0.0, front=0.1, nav_active=False))
    assert not out.override


# --- Obstacle detected -> stop

def test_obstacle_ahead_stops_robot():
    out = ReactiveFsm(CFG).step(inputs(0.0, front=0.3))
    assert out.state == State.STOPPED and out.override and out.v == 0.0 and out.w == 0.0


def test_reversing_or_rotating_is_not_stopped():
    assert ReactiveFsm(CFG).step(inputs(0.0, front=0.3, cmd_v=-0.2)).state == State.NAVIGATING
    assert ReactiveFsm(CFG).step(inputs(0.0, front=0.3, cmd_v=0.0)).state == State.NAVIGATING


def test_obstacle_beyond_goal_is_ignored():
    assert ReactiveFsm(CFG).step(inputs(0.0, front=0.3, dist_to_goal=0.2)).state \
        == State.NAVIGATING


def test_hysteresis_stays_stopped_between_thresholds():
    fsm = stopped_fsm()
    assert fsm.step(inputs(1.0, front=0.40)).state == State.STOPPED


def test_stays_stopped_for_minimum_hold():
    fsm = stopped_fsm()
    assert fsm.step(inputs(0.1, front=1.0)).state == State.STOPPED


def test_resumes_when_front_clears():
    fsm = stopped_fsm()
    out = fsm.step(inputs(1.0, front=0.6))
    assert out.state == State.NAVIGATING and not out.override


def test_stop_times_out_into_replan():
    fsm = stopped_fsm()
    out = fsm.step(inputs(CFG.stop_timeout, front=0.3))
    assert out.state == State.REPLANNING and out.request_replan


# --- Obstacle on one side -> turn toward the safer direction

def test_side_obstacle_left_turns_right():
    out = ReactiveFsm(CFG).step(inputs(0.0, left=0.2, right=1.0))
    assert out.state == State.TURNING and out.w < 0 and out.v == 0.0


def test_side_obstacle_right_turns_left():
    out = ReactiveFsm(CFG).step(inputs(0.0, left=1.0, right=0.2))
    assert out.state == State.TURNING and out.w > 0


def test_narrow_gap_with_both_sides_equal_does_not_turn():
    assert ReactiveFsm(CFG).step(inputs(0.0, left=0.2, right=0.22)).state == State.NAVIGATING


def test_no_side_turn_near_goal():
    assert ReactiveFsm(CFG).step(inputs(0.0, left=0.2, right=1.0, dist_to_goal=0.3)).state \
        == State.NAVIGATING


def test_blocked_ahead_with_safer_side_turns():
    fsm = stopped_fsm()
    out = fsm.step(inputs(CFG.stop_hold, front=0.3, left=0.3, right=1.0))
    assert out.state == State.TURNING and out.w < 0


def test_turn_direction_is_locked_while_turning():
    fsm = ReactiveFsm(CFG)
    first = fsm.step(inputs(0.0, left=0.2, right=1.0))
    later = fsm.step(inputs(0.5, left=1.0, right=0.2))
    assert later.w == first.w


def test_turn_ends_when_clear():
    fsm = ReactiveFsm(CFG)
    fsm.step(inputs(0.0, left=0.2, right=1.0))
    assert fsm.step(inputs(1.0)).state == State.NAVIGATING


def test_turn_times_out_into_replan():
    fsm = ReactiveFsm(CFG)
    fsm.step(inputs(0.0, left=0.2, right=1.0))
    out = fsm.step(inputs(CFG.turn_timeout, left=0.2, right=1.0))
    assert out.state == State.REPLANNING and out.request_replan


# --- Planned path blocked -> trigger re-planning

def test_blocked_path_requests_exactly_one_replan():
    fsm = replanning_fsm()
    out = fsm.step(inputs(0.1, path_valid=False, replan_status=ReplanStatus.PENDING))
    assert out.state == State.REPLANNING and out.override and not out.request_replan


def test_blocked_path_while_stopped_triggers_replan():
    fsm = stopped_fsm()
    out = fsm.step(inputs(CFG.stop_hold, front=0.3, path_valid=False))
    assert out.state == State.REPLANNING and out.request_replan


def test_failed_planner_is_retried_after_delay():
    fsm = replanning_fsm()
    assert not fsm.step(inputs(0.5, path_valid=False, replan_status=ReplanStatus.FAILED)) \
        .request_replan
    assert fsm.step(inputs(CFG.replan_retry, path_valid=False,
                           replan_status=ReplanStatus.FAILED)).request_replan


# --- New path available -> resume navigation

def test_resumes_after_valid_new_path_and_settle_time():
    fsm = replanning_fsm()
    fsm.step(inputs(0.1, replan_status=ReplanStatus.SUCCEEDED))
    assert fsm.step(inputs(0.5, replan_status=ReplanStatus.SUCCEEDED)).state == State.REPLANNING
    out = fsm.step(inputs(0.1 + CFG.replan_settle, replan_status=ReplanStatus.SUCCEEDED))
    assert out.state == State.NAVIGATING and not out.override


def test_does_not_resume_on_invalid_new_path():
    fsm = replanning_fsm()
    fsm.step(inputs(0.1, path_valid=False, replan_status=ReplanStatus.SUCCEEDED))
    assert fsm.step(inputs(2.0, path_valid=False, replan_status=ReplanStatus.SUCCEEDED)).state \
        == State.REPLANNING


def test_replan_timeout_hands_back_to_nav2():
    fsm = replanning_fsm()
    out = fsm.step(inputs(CFG.replan_timeout, path_valid=False,
                          replan_status=ReplanStatus.FAILED))
    assert out.state == State.NAVIGATING and not out.override


def test_cooldown_prevents_immediate_second_replan():
    fsm = replanning_fsm()
    fsm.step(inputs(CFG.replan_timeout, path_valid=False))
    assert fsm.step(inputs(CFG.replan_timeout + 0.5, path_valid=False)).state == State.NAVIGATING


def test_grace_period_lets_nav2_steer_away():
    fsm = stopped_fsm()
    fsm.step(inputs(1.0, front=0.6))
    assert fsm.step(inputs(1.5, front=0.3)).state == State.NAVIGATING   # allowed in grace
    assert fsm.step(inputs(1.6, front=0.1)).state == State.STOPPED      # but not this close


def test_after_grace_normal_stop_applies():
    fsm = stopped_fsm()
    fsm.step(inputs(1.0, front=0.6))
    assert fsm.step(inputs(1.0 + CFG.grace_period + 0.1, front=0.3)).state == State.STOPPED


def test_goal_ending_clears_override():
    fsm = stopped_fsm()
    assert not fsm.step(inputs(0.1, front=0.3, nav_active=False)).override
    assert fsm.state == State.NAVIGATING
