import os
import random

# Directory to save generated world files
output_dir = os.getcwd()

# Number of random worlds to generate
num_worlds = 100
# Number of rocks in a 30m radius of gps coordinates
num_rocks = 81
r = 30

def generate_random_coordinates(center_x, center_y, radius, min_distance, count):
    coordinates = []
    while len(coordinates) < count:
        x = random.uniform(center_x - radius, center_x + radius)
        y = random.uniform(center_y - radius, center_y + radius)
        distance = ((x - center_x)**2 + (y - center_y)**2)**0.5
        if distance >= min_distance and distance <= radius and ((x**2 + y**2)**0.5 >= 3):
            coordinates.append((x, y))
    return coordinates

def generate_ar_tag_coordinates(center_x, center_y, radius, count):
    coordinates = []
    while len(coordinates) < count:
        x = random.uniform(center_x - radius, center_x + radius)
        y = random.uniform(center_y - radius, center_y + radius)
        distance = ((x - center_x)**2 + (y - center_y)**2)**0.5
        if distance <= radius:
            coordinates.append((x, y))
    return coordinates

def generate_world_file(file_index):
    # Base content for the world
    base_content = """<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
  <world name=\"default\">

    <include>
      <uri>model://custom_ground</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>38.161479</latitude_deg>
      <longitude_deg>-122.454630</longitude_deg>
      <elevation>488.0</elevation>
      <heading_deg>90</heading_deg>
    </spherical_coordinates>

"""

    # Cylinder positions
    cylinder_positions = [
        (10, 0),
        (40, 40),
        (80, 80),
        (120, 40), # 1
        (160, 0),
        (200, -40),
        (240, -80), # 2
        (280, -40), 
        (320, 0),
        (360, 40), # 3
        (400, 80),
        (440, 40),
        (480, 0), # 4
        (520, -40),
        (560, -80),
        (600, -40), # 5
        (640, 0),
        (680, 40),
        (720, 80), # 6
        (760, 40),
        (800, 0),
        (840, -40) # 7
    ]

    rock_models = ""
    for i, (cx, cy) in enumerate(cylinder_positions):
        rock1_coords = generate_random_coordinates(cx, cy, r, 2, num_rocks // 3)
        rock2_coords = generate_random_coordinates(cx, cy, r, 2, num_rocks // 3)
        rock3_coords = generate_random_coordinates(cx, cy, r, 2, num_rocks // 3)

        for j, (x, y) in enumerate(rock1_coords):
            rock_models += f"    <model name=\"rock1_{i}_{j}\">\n      <include>\n        <uri>model://rock1</uri>\n        <pose>{x} {y} 0.5 0 0 0</pose>\n      </include>\n    </model>\n"

        for j, (x, y) in enumerate(rock2_coords):
            rock_models += f"    <model name=\"rock2_{i}_{j}\">\n      <include>\n        <uri>model://rock2</uri>\n        <pose>{x} {y} 0.3 0 0 0</pose>\n      </include>\n    </model>\n"

        for j, (x, y) in enumerate(rock3_coords):
            rock_models += f"    <model name=\"rock3_{i}_{j}\">\n      <include>\n        <uri>model://rock3</uri>\n        <pose>{x} {y} 0.2 0 0 0</pose>\n      </include>\n    </model>\n"

    base_content += rock_models

    # Add AR tags around specific cylinders
    ar_tag_specs = [
        ((40, 40), "ar_tag1", 10.0),
        ((80, 80), "ar_tag2", 15.0),
        ((120, 40), "ar_tag3", 20.0),
        ((160, 0), "ar_tag1", 10.0),
        ((200, -40), "ar_tag2", 15.0),
        ((240, -80), "ar_tag3", 20.0),
        ((280, -40), "ar_tag1", 10.0),
        ((320, 0), "ar_tag2", 15.0),
        ((360, 40), "ar_tag3", 20.0),
        ((400, 80), "ar_tag1", 10.0),
        ((440, 40), "ar_tag2", 15.0),
        ((480, 0), "ar_tag3", 20.0),
        ((520, -40), "ar_tag1", 10.0),
        ((560, -80), "ar_tag2", 15.0),
        ((600, -40), "ar_tag3", 20.0),
        ((640, 0), "ar_tag1", 10.0),
        ((680, 40), "ar_tag2", 15.0),
        ((720, 80), "ar_tag3", 20.0),
        ((760, 40), "ar_tag1", 10.0),
        ((800, 0), "ar_tag2", 15.0),
        ((840, -40), "ar_tag3", 20.0)
    ]

    ar_tag_models = ""
    for i, ((cx, cy), tag_name, radius) in enumerate(ar_tag_specs):
        ar_coords = generate_ar_tag_coordinates(cx, cy, radius, count=1)  # Generate 5 tags each
        for j, (x, y) in enumerate(ar_coords):
            ar_tag_models += f"    <model name=\"{tag_name}_{i}_{j}\">\n      <include>\n        <uri>model://{tag_name}</uri>\n        <pose>{x} {y} 0.0 0 0 0</pose>\n      </include>\n    </model>\n"

    base_content += ar_tag_models

    # Add cylinder models
    for i, (cx, cy) in enumerate(cylinder_positions):
        base_content += f"""
    <model name=\"cylinder_{i+1}\">
      <static>true</static>
      <link name=\"link\">
        <visual name=\"visual\">
          <geometry>
            <cylinder>
              <radius>1</radius>
              <length>500</length>
            </cylinder>
          </geometry>
        </visual>
      </link>
      <pose>{cx} {cy} 270 0 0 0</pose>
    </model>

"""

    base_content += "  </world>\n</sdf>"

    # Write the content to a file
    filename = os.path.join(output_dir, f"randworld_condensed{file_index}.world")
    with open(filename, "w") as f:
        f.write(base_content)

# Generate 100 world files
for i in range(1, num_worlds + 1):
    generate_world_file(i)

print(f"Generated 100 world files in {output_dir}")
