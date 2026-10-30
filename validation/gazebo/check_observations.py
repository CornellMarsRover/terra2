"""Match ROS costmap cells to physical Gazebo obstacle footprints."""
import argparse
import json
import math
import xml.etree.ElementTree as ET

from shapely.affinity import rotate, translate
from shapely.geometry import Point, box

parser = argparse.ArgumentParser()
parser.add_argument('world')
parser.add_argument('telemetry')
parser.add_argument('--output')
args = parser.parse_args()
shapes = {}
for model in ET.parse(args.world).getroot().iter('model'):
    geometry = model.find('./link/collision/geometry')
    if geometry is None:
        continue
    pose = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
    block = geometry.find('box')
    if block is not None:
        sx, sy, _ = map(float, block.findtext('size').split())
        shape = translate(rotate(box(-sx/2, -sy/2, sx/2, sy/2),
                                 pose[5], use_radians=True), pose[0], pose[1])
    else:
        radius = geometry.findtext('cylinder/radius') or geometry.findtext('sphere/radius')
        if not radius:
            continue
        shape = Point(pose[:2]).buffer(float(radius))
    shapes[model.attrib['name']] = shape
seen = {name: {'nearest_cost_cell_m': math.inf, 'first_seen_sim_s': None}
        for name in shapes}
count = 0
with open(args.telemetry, encoding='utf-8') as stream:
    for line in stream:
        event = json.loads(line)
        if event['topic'] != 'costs':
            continue
        count += 1
        if count % 10:
            continue
        cells = event['data']
        for x, y, cost in zip(cells[::3], cells[1::3], cells[2::3]):
            if cost < 10:
                continue
            point = Point(x, y)
            for name, shape in shapes.items():
                if abs(x-shape.centroid.x) > 5 or abs(y-shape.centroid.y) > 5:
                    continue
                distance = point.distance(shape)
                if distance < seen[name]['nearest_cost_cell_m']:
                    seen[name]['nearest_cost_cell_m'] = distance
                if distance <= 0.6 and seen[name]['first_seen_sim_s'] is None:
                    seen[name]['first_seen_sim_s'] = event['time_ns'] * 1e-9
for result in seen.values():
    if math.isinf(result['nearest_cost_cell_m']):
        result['nearest_cost_cell_m'] = None
    result['costmap_seen'] = result['first_seen_sim_s'] is not None
text = json.dumps(seen, indent=2)
print(text)
if args.output:
    with open(args.output, 'w', encoding='utf-8') as stream:
        stream.write(text + '\n')
