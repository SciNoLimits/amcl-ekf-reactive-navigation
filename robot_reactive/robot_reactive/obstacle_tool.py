"""Add, move and remove box obstacles in the running Gazebo world.

All positions are in the Nav2 *map* frame, the same numbers you see in RViz.
Gazebo's world frame is offset from it because the map was built with the robot
at its spawn point: world = map + (-6, +2).

Command line examples (with the simulation running):
    ros2 run robot_reactive obstacle_tool spawn --name box1 --x 2.0 --y -1.0
    ros2 run robot_reactive obstacle_tool move --name box1 --x 2.0 --y 0.0
    ros2 run robot_reactive obstacle_tool shuttle --name box1 --x 2.2 --y -2.8 --y2 1.0
    ros2 run robot_reactive obstacle_tool remove --name box1
"""
import argparse
import subprocess
import time

WORLD = 'default'
MAP_TO_WORLD = (-6.0, 2.0)


def _call(service: str, reqtype: str, request: str, timeout_ms: int = 3000) -> bool:
    result = subprocess.run(
        ['gz', 'service', '-s', f'/world/{WORLD}/{service}', '--reqtype', reqtype,
         '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms), '--req', request],
        capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 5.0)
    return 'data: true' in result.stdout


def spawn_box(name, x, y, sx=0.4, sy=0.4, sz=0.6) -> bool:
    geometry = f'<box><size>{sx} {sy} {sz}</size></box>'
    sdf = (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'>"
           f"<static>true</static><link name='link'>"
           f"<collision name='c'><geometry>{geometry}</geometry></collision>"
           f"<visual name='v'><geometry>{geometry}</geometry>"
           f"<material><ambient>0.9 0.3 0.1 1</ambient><diffuse>0.9 0.3 0.1 1</diffuse>"
           f"</material></visual></link></model></sdf>")
    world_x, world_y = x + MAP_TO_WORLD[0], y + MAP_TO_WORLD[1]
    request = (f'sdf: "{sdf}" name: "{name}" '
               f'pose: {{position: {{x: {world_x}, y: {world_y}, z: {sz / 2.0}}}}}')
    return _call('create', 'gz.msgs.EntityFactory', request)


def move_box(name, x, y, sz=0.6) -> bool:
    world_x, world_y = x + MAP_TO_WORLD[0], y + MAP_TO_WORLD[1]
    request = (f'name: "{name}" '
               f'position: {{x: {world_x}, y: {world_y}, z: {sz / 2.0}}} '
               f'orientation: {{w: 1.0}}')
    return _call('set_pose', 'gz.msgs.Pose', request)


def remove_box(name) -> bool:
    return _call('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')


def shuttle_positions(y1: float, y2: float, speed: float, t: float) -> float:
    """Position at time t for something moving back and forth between y1 and y2."""
    span = abs(y2 - y1)
    phase = (speed * t) % (2.0 * span)
    travelled = phase if phase <= span else 2.0 * span - phase
    return y1 + travelled * (1.0 if y2 > y1 else -1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['spawn', 'move', 'remove', 'shuttle'])
    parser.add_argument('--name', default='box1')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--y2', type=float, default=0.0, help='shuttle: other end of the track')
    parser.add_argument('--sx', type=float, default=0.4)
    parser.add_argument('--sy', type=float, default=0.4)
    parser.add_argument('--speed', type=float, default=0.4, help='shuttle: speed in m/s')
    args = parser.parse_args()

    if args.command == 'spawn':
        print('ok' if spawn_box(args.name, args.x, args.y, args.sx, args.sy) else 'failed')
    elif args.command == 'move':
        print('ok' if move_box(args.name, args.x, args.y) else 'failed')
    elif args.command == 'remove':
        print('ok' if remove_box(args.name) else 'failed')
    else:
        spawn_box(args.name, args.x, args.y, args.sx, args.sy)
        start = time.monotonic()
        try:
            while True:
                y = shuttle_positions(args.y, args.y2, args.speed, time.monotonic() - start)
                move_box(args.name, args.x, y)
                time.sleep(0.05)
        except KeyboardInterrupt:
            remove_box(args.name)


if __name__ == '__main__':
    main()
