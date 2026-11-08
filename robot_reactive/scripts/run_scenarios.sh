#!/bin/bash
# Run test scenarios, each in a freshly launched simulation so runs can't affect
# each other. Run inside the ROS container, from the workspace root:
#
#   src/amcl-ekf-reactive-navigation/robot_reactive/scripts/run_scenarios.sh S1:on S2:on S2:off
#
# Each argument is SCENARIO:label. Labels starting with "off" turn the reactive
# layer off; add a number to repeat a run, e.g. S5B:on1 S5B:on2 S5B:off1.
# Results go to $OUT (default: the package's results/ folder).

WS=${WS:-/home/ros/jazzy_ws}
OUT=${OUT:-$WS/src/amcl-ekf-reactive-navigation/robot_reactive/results}
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
mkdir -p "$OUT"

stop_everything() {
    pkill -INT -f "ros2 launch robot_bringup" 2>/dev/null
    sleep 4
    pkill -9 -f "gz sim|gz_tools_vendor|component_container|rviz2|parameter_bridge|robot_state_publisher|obstacle_detector|reactive_controller|spawner" 2>/dev/null
    sleep 2
}

trap stop_everything EXIT

for job in "$@"; do
    scenario=${job%%:*}
    label=${job##*:}
    case "$label" in off*) enable=false ;; *) enable=true ;; esac
    echo "=== $scenario reactive=$label"

    stop_everything
    log=/tmp/run_${scenario}_${label}.log
    setsid ros2 launch robot_bringup bringup.launch.py enable_reactive:=$enable >"$log" 2>&1 &

    for _ in $(seq 1 120); do
        grep -aq "Managed nodes are active" "$log" && break
        sleep 1
    done
    sleep 8     # let AMCL publish its first pose and the costmaps fill in

    timeout 260 ros2 run robot_reactive scenario_runner "$scenario" --label "$label" --out "$OUT"
    echo "exit code: $?"
done
