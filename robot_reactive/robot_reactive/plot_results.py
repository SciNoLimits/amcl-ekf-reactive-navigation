"""Draw top-down plots from scenario_runner's JSON files.

Usage:
    ros2 run robot_reactive plot_results <results_dir> <map.yaml>

Writes <results_dir>/plots/<scenario>_<label>.png: the map, the planned paths,
the route actually driven (coloured by reactive state) and any obstacles.
"""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt     # noqa: E402
from matplotlib.lines import Line2D     # noqa: E402
from matplotlib.patches import Rectangle    # noqa: E402
import numpy as np      # noqa: E402

STATE_STYLE = {
    'NAVIGATING': ('#2b6cb0', 'Navigating'),
    'STOPPED': ('#c53030', 'Stopped'),
    'TURNING': ('#b7791f', 'Turning'),
    'REPLANNING': ('#6b46c1', 'Replanning'),
}
DISABLED_COLOUR = '#4a5568'


def read_map(yaml_path: Path):
    meta = {}
    for line in yaml_path.read_text().splitlines():
        if ':' in line:
            key, value = line.split(':', 1)
            meta[key.strip()] = value.strip()
    with open(yaml_path.parent / meta['image'], 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = map(int, line.split())
        f.readline()
        image = np.frombuffer(f.read(), dtype=np.uint8).reshape(height, width)
    origin = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    resolution = float(meta['resolution'])
    extent = [origin[0], origin[0] + width * resolution,
              origin[1], origin[1] + height * resolution]
    return image, extent


def plot(result: dict, image, extent, out: Path):
    fig, ax = plt.subplots(figsize=(6.4, 5.0), dpi=170)
    ax.imshow(image, cmap='gray', vmin=0, vmax=255, extent=extent, alpha=0.55)

    plans = result['plans']
    if plans:
        first, last = plans[0]['points'], plans[-1]['points']
        ax.plot(*zip(*first), ls='--', lw=1.2, color='#718096')
        ax.plot(*zip(*last), ls=':', lw=1.2, color='#2f855a')
    for alt in result['alt_paths'][:1]:
        ax.plot(*zip(*alt['points']), lw=1.6, color='#6b46c1', alpha=0.85)

    for cx, cy, sx, sy in result['boxes'].values():
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color='#dd6b20', alpha=0.85))

    traj = result['trajectory']
    for a, b in zip(traj, traj[1:]):
        colour = STATE_STYLE.get(a['state'], (DISABLED_COLOUR, ''))[0]
        ax.plot([a['x'], b['x']], [a['y'], b['y']], color=colour, lw=2.2,
                solid_capstyle='round')
    ax.plot(traj[0]['x'], traj[0]['y'], 'o', color='#1a202c', ms=5)
    ax.plot(*result['goal'], '*', color='#d69e2e', mec='#1a202c', mew=0.5, ms=13)

    used = {p['state'] for p in traj}
    handles = [Line2D([], [], color=STATE_STYLE[s][0], lw=2.2, label=STATE_STYLE[s][1])
               for s in STATE_STYLE if s in used]
    if 'DISABLED' in used:
        handles.append(Line2D([], [], color=DISABLED_COLOUR, lw=2.2, label='Driven route'))
    handles += [Line2D([], [], color='#718096', ls='--', label='First plan'),
                Line2D([], [], color='#2f855a', ls=':', label='Last plan')]
    if result['alt_paths']:
        handles.append(Line2D([], [], color='#6b46c1', lw=1.6, label='Replanned path'))
    ax.legend(handles=handles, loc='upper right', fontsize=7, framealpha=0.95)

    label = 'reactive on' if result['label'] == 'on' else 'reactive off'
    ax.set_title(f"{result['scenario']}: {result['description']} ({label}): {result['status']}",
                 fontsize=9)
    ax.set_xlabel('map x (m)', fontsize=8)
    ax.set_ylabel('map y (m)', fontsize=8)
    ax.tick_params(labelsize=7)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def main():
    results_dir, map_yaml = Path(sys.argv[1]), Path(sys.argv[2])
    image, extent = read_map(map_yaml)
    out_dir = results_dir / 'plots'
    out_dir.mkdir(parents=True, exist_ok=True)
    for path in sorted(results_dir.glob('*.json')):
        plot(json.loads(path.read_text()), image, extent, out_dir / f'{path.stem}.png')
        print(f'wrote {path.stem}.png')


if __name__ == '__main__':
    main()
