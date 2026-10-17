#!/usr/bin/env python3
"""Export tables and a map figure from actual saved experiment artifacts."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import yaml

root = Path(__file__).resolve().parents[2]
results = root / 'part1_rosorin/results/runtime'
rows = []
for filename in ('navigation_trials.json', 'navigation_trials_demo.json'):
    payload = json.loads((results / filename).read_text())
    for trial in payload['trials']:
        rows.append({'run': filename, 'task': trial['name'],
                     'success': trial['pass'], 'action_result': trial['action_result'],
                     'waypoints_requested': len(trial['goals']),
                     'missed_waypoints': len(trial.get('missed_waypoints', [])),
                     'wall_seconds': trial['wall_seconds'],
                     'simulation_seconds': trial['simulation_seconds'],
                     'final_position_error_m': trial['final_position_error_m']})
with (results / 'navigation_summary.csv').open('w', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
summary = {}
for name in ('room_slam', 'room_demo'):
    metadata = yaml.safe_load((root / f'part1_rosorin/maps/{name}.yaml').read_text())
    array = np.asarray(Image.open(root / f'part1_rosorin/maps/{name}.pgm'))
    summary[name] = {'width': array.shape[1], 'height': array.shape[0],
                     'resolution_m': metadata['resolution'], 'origin': metadata['origin'],
                     'free_cells': int((array == 254).sum()),
                     'occupied_cells': int((array == 0).sum()),
                     'unknown_cells': int((array == 205).sum()),
                     'total_cells': int(array.size)}
    if name == 'room_demo':
        resolution = metadata['resolution']
        x, y = metadata['origin'][:2]
        extent = [x, x + array.shape[1] * resolution, y, y + array.shape[0] * resolution]
        fig, ax = plt.subplots(figsize=(7, 6))
        ax.imshow(array, cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper')
        demo = json.loads((results / 'navigation_trials_demo.json').read_text())
        for trial in demo['trials']:
            goals = np.asarray(trial['goals'])
            ax.scatter(goals[:, 0], goals[:, 1], marker='x', s=65, label=trial['name'] + ' targets')
            final = trial['final_pose']
            ax.scatter(final[0], final[1], marker='o', s=30, label=trial['name'] + ' measured final')
        ax.set(xlabel='Map x (m)', ylabel='Map y (m)', title='SLAM map and measured navigation endpoints')
        ax.legend(fontsize=8, loc='upper left')
        fig.tight_layout()
        fig.savefig(results / 'map_navigation_results.png', dpi=160)
        plt.close(fig)
(results / 'map_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({'trials': len(rows), 'successes': sum(r['success'] for r in rows),
                  'map_summary': summary}, indent=2))
