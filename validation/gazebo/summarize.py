"""Compact evidence for humans and agents; video success is separate from navigation."""
import json
import math
import pathlib
import subprocess
import sys

folder = pathlib.Path(sys.argv[1])
report = json.loads((folder / 'report.json').read_text())
contacts = sum(o['intersection_samples'] for o in report['obstacles'].values())
goals = report['goal_minimum_m']
reached = all(distance <= 1.0 for distance in goals.values())
finish = tuple(map(float, next(reversed(goals)).split(',')))
final_distance = math.dist(report['final_pose'], finish)
mission_complete = 'All waypoints reached.' in (folder / 'state.log').read_text()
probe = subprocess.run(['ffprobe', '-v', 'error', '-show_entries',
                        'format=duration', '-of', 'csv=p=0',
                        str(folder / 'demo.mp4')], capture_output=True, text=True)
video_ok = probe.returncode == 0 and float(probe.stdout or 0) > 1
passed = reached and final_distance <= 1.0 and mission_complete and contacts == 0 and report['samples'] > 10 and video_ok
text = '\n'.join([
    f"{'PASS' if passed else 'INCOMPLETE/FAIL'}: recorded navigation acceptance",
    f"Samples: {report['samples']}; sim seconds: {report['duration_s']:.1f}",
    f"Travel: {report['path_length_m']:.2f} m; footprint contact samples: {contacts}",
    f'Goal minimum distances (1 m threshold): {goals}',
    f'Finish distance: {final_distance:.2f} m; mission complete: {mission_complete}',
    f"Video: {'OK' if video_ok else 'FAILED'} demo.mp4",
    'Evidence: report.json, odom.csv, state.log, planner.log, gazebo.log',
    'Planar footprint check; does not establish wheel/contact dynamics.',
]) + '\n'
(folder / 'summary.txt').write_text(text)
print(text, end='')
sys.exit(0 if passed else 1)
