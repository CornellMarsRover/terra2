#!/usr/bin/env python3
import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET

from shapely.affinity import rotate, translate
from shapely.geometry import Point, box


def rectangle(x, y, sx, sy, yaw):
    shape = box(-sx / 2, -sy / 2, sx / 2, sy / 2)
    return translate(rotate(shape, yaw, use_radians=True), x, y)


parser = argparse.ArgumentParser()
parser.add_argument("world")
parser.add_argument("odometry")
parser.add_argument("--output")
parser.add_argument("--goal", action="append", default=["5,0", "10,10"])
args = parser.parse_args()
poses = []
for row in csv.reader(open(args.odometry, encoding="utf-8")):
    try:
        yaw = 2 * math.atan2(float(row[9]), float(row[10]))
        poses.append((int(row[0]) + int(row[1]) * 1e-9, float(row[4]), float(row[5]), yaw))
    except (IndexError, ValueError):
        continue

obstacles = {}
for model in ET.parse(args.world).getroot().iter("model"):
    geometry = model.find("./link/collision/geometry")
    if geometry is None:
        continue
    values = [float(value) for value in model.findtext("pose", "0 0 0 0 0 0").split()]
    shape = geometry.find("box")
    if shape is not None:
        sx, sy, _ = map(float, shape.findtext("size").split())
        obstacles[model.attrib["name"]] = rectangle(values[0], values[1], sx, sy, values[5])
        continue
    shape = geometry.find("cylinder")
    radius = shape.findtext("radius") if shape is not None else geometry.findtext("sphere/radius")
    if radius:
        obstacles[model.attrib["name"]] = Point(values[0], values[1]).buffer(float(radius))

result = {"samples": len(poses)}
result["duration_s"] = poses[-1][0] - poses[0][0]
result["path_length_m"] = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(poses, poses[1:]))
result["final_pose"] = poses[-1][1:3]
result["goal_minimum_m"] = {
    goal: min(math.dist(pose[1:3], tuple(map(float, goal.split(",")))) for pose in poses)
    for goal in args.goal
}
result["obstacles"] = {}
for name, obstacle in obstacles.items():
    distances = [
        rectangle(x, y, 1.08, 1.14, yaw).distance(obstacle)
        for _, x, y, yaw in poses
    ]
    result["obstacles"][name] = {
        "minimum_clearance_m": min(distances),
        "intersection_samples": sum(distance <= 1e-6 for distance in distances),
    }
text = json.dumps(result, indent=2)
print(text)
if args.output:
    open(args.output, "w", encoding="utf-8").write(text + "\n")
