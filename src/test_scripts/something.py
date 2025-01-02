import qi  # For interacting with Pepper's APIs
import numpy as np
from PIL import Image
import yaml

# Load YAML metadata (your map's metadata)
with open('/home/humanoidrobots/test_scripts/maps/room.yaml', 'r') as file:
    map_metadata = yaml.load(file)  # Python 2 style loading

# Path to the PGM image file from YAML data
image_path = map_metadata['image']

# Load the PGM image using PIL
img = Image.open(image_path)

# Get the map resolution and size from YAML metadata
resolution = map_metadata['resolution']  # e.g., 0.05 meters per pixel
origin = map_metadata['origin']  # [x, y, z] coordinates of the origin

# Set custom threshold values for occupied and free space
occupied_thresh = 0.2  # Occupied space is from 0 to 0.2 (lower values)

# Convert the full map to a NumPy array
img_array = np.array(img)

# Normalize the image array to values between 0 and 1
img_array_normalized = img_array / 255.0

# Find the occupied areas (those below occupied_thresh)
occupied_map = img_array_normalized <= occupied_thresh

# Initialize visited array to track visited pixels
visited = np.zeros_like(occupied_map)

# Function to perform flood fill to find connected components
def flood_fill(occupied_map, visited, start_row, start_col):
    stack = [(start_row, start_col)]
    component = []
    while stack:
        row, col = stack.pop()
        if visited[row, col] == 0:  # If the pixel is not visited
            visited[row, col] = 1
            component.append((row, col))
            # Add all 4-neighbor pixels to the stack
            for r, c in [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]:
                if 0 <= r < occupied_map.shape[0] and 0 <= c < occupied_map.shape[1]:
                    if occupied_map[r, c] and visited[r, c] == 0:
                        stack.append((r, c))
    return component

# Find all the objects (connected components)
objects = []
for row in range(occupied_map.shape[0]):
    for col in range(occupied_map.shape[1]):
        if occupied_map[row, col] and visited[row, col] == 0:
            # Start flood fill from unvisited occupied pixel
            component = flood_fill(occupied_map, visited, row, col)
            objects.append(component)

# Get the 116th object (object ID starts from 1, so index 115 for 116th object)
object_id = 116
if object_id <= len(objects):
    component = objects[object_id - 1]  # Indexing starts from 0
    rows, cols = zip(*component)
    min_row, max_row = min(rows), max(rows)
    min_col, max_col = min(cols), max(cols)

    # Convert the pixel coordinates of the bounding box to real-world coordinates
    x_min = min_col * resolution + origin[0]
    y_min = min_row * resolution + origin[1]
    x_max = max_col * resolution + origin[0]
    y_max = max_row * resolution + origin[1]

    # Calculate the center of the object
    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2

    print("Real-world coordinates of object 116 center: X = {:.2f}, Y = {:.2f}".format(x_center, y_center))

    # Connect to Pepper and move it to the object's coordinates
    try:
        # Connect to Pepper (replace with correct IP address of your robot)
        session = qi.Session()
        session.connect("tcp://192.168.0.102:9559")

        # Get the motion proxy
        motion_proxy = session.service("ALMotion")
        # Get the robot's current position (optional, to avoid collision with obstacles)
        current_position = motion_proxy.getRobotPosition(True)

        # Define the target position (x_center, y_center)
        target_position = [x_center, y_center, current_position[2]]  # Keep the robot's orientation unchanged

        # Move the robot to the target position
        motion_proxy.moveTo(target_position[0], target_position[1], target_position[2])

        print("Pepper is moving to object 116 at coordinates: X = {:.2f}, Y = {:.2f}".format(x_center, y_center))

    except Exception as e:
        print("Error connecting to Pepper: ", str(e))
else:
    print("Object 116 not found.")