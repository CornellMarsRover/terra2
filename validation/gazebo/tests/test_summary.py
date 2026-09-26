"""Acceptance must distinguish a produced video from successful navigation."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('distance,contacts,video,expected', [
    (1.0, 0, True, 0), (2.1, 0, True, 1),
    (1.0, 1, True, 1), (1.0, 0, False, 1),
])
def test_acceptance(tmp_path, distance, contacts, video, expected):
    report = dict(samples=20, duration_s=4.0, path_length_m=3.0,
                  final_pose=[distance, 0], goal_minimum_m={'0,0': distance},
                  obstacles={'wall': {'intersection_samples': contacts}})
    (tmp_path / 'report.json').write_text(json.dumps(report))
    (tmp_path / 'state.log').write_text('All waypoints reached.')
    probe = tmp_path / 'ffprobe'
    probe.write_text('#!/bin/sh\n' + ('echo 3.0\n' if video else 'exit 1\n'))
    probe.chmod(0o755)
    script = Path(__file__).resolve().parents[1] / 'summarize.py'
    result = subprocess.run([sys.executable, str(script), str(tmp_path)],
                            env={**os.environ, 'PATH': str(tmp_path)},
                            capture_output=True, text=True)
    assert result.returncode == expected, result.stderr
    assert (tmp_path / 'summary.txt').is_file()
